using System;
using System.Threading;
using AutonomousCar.Entities;
using C5;
using FarseerPhysics.DebugViews;
using Microsoft.Xna.Framework;

namespace AutonomousCar.PathFinding
{
    /// <summary>
    /// The Smoother class generates a smoothed path based on the path discovered by the path planner.
    /// This class used iterative gradient descent to optimize the path with regards to several
    /// terms: smoothness, proximity to obstacles, proximity to the Voronoi edge, and curvature.
    /// </summary>
    public static class Smoother
    {
        public static float Tolerance = 0.001f;
        public static int MaxIterations = 2000;
        public static float Alpha = 0.2f;
        public static float PathWeight = 0f;
        public static float SmoothWeight = 4f;
        public static float CollisionWeight = 0.002f;
        public static float CollisionDmax = 5f;
        public static float VoronoiWeight = 0.2f;
        public static float VoronoiDmax = 20f;
        public static float CurvatureWeight = 4f;
        public static float KMax = 1f / (Car.TURNING_RADIUS * 1.1f);

        public static int NumIterations { get; private set; }
        public static float Change { get; private set; }
        public static HashSet<int> UnsafeIndices { get; private set; }
        public static bool Anchoring = true;
        public static int Delay = 0;

        private static Pose[] currentPath;
        private static int currentCount;

        public static ArrayList<Pose> Smooth(ArrayList<Pose> path, ObstacleGrid grid)
        {
            return Smooth(path, grid, null);
        }

        public static ArrayList<Pose> Smooth(ArrayList<Pose> path, ObstacleGrid grid, HashSet<int> unsafeIndices)
        {
            UnsafeIndices = Anchoring ? unsafeIndices : null;

            if (path.Count < 5)
            {
                ArrayList<Pose> thepath = new ArrayList<Pose>();
                thepath.AddAll(path);
                return thepath;
            }

            int num = currentCount = path.Count;

            currentPath = new Pose[num];
            for (int i = 0; i < num; i++)
                currentPath[i] = path[i];

            int count = 0;
            float change = Tolerance;
            while (/*change >= Tolerance && */count < MaxIterations)
            {
                float cdmax2 = CollisionDmax * CollisionDmax;
                change = 0f;
                for (int i = 2; i < num - 2; i++)
                {
                    // Keep this point fixed if it is or is adjacent to a cusp point
                    if (currentPath[i - 2].Gear != currentPath[i - 1].Gear) continue;
                    if (currentPath[i - 1].Gear != currentPath[i].Gear) continue;
                    if (currentPath[i].Gear != currentPath[i + 1].Gear) continue;
                    if (currentPath[i + 1].Gear != currentPath[i + 2].Gear) continue;

                    // Keep this point fixed if it is unsafe
                    if (Anchoring && unsafeIndices != null && unsafeIndices.Contains(i))
                        continue;

                    Vector2 curr = currentPath[i].Position;
                    Vector2 correction = Vector2.Zero;
                    
                    // Original path term
                    correction += PathWeight * (path[i].Position - curr);
                    if (float.IsNaN(correction.X) || float.IsNaN(correction.Y))
                    {
                        float noop;
                    }

                    if (grid.IsPointInGrid(curr))
                    {
                        // Collision term
                        Vector2 closestObstacleVec = curr - grid.CellPositionToPoint(grid.GVD.GetNearestObstacle(grid.PointToCellPosition(curr)));
                        float obstDist = closestObstacleVec.Length();
                        if (obstDist < CollisionDmax)
                            correction -= CollisionWeight * (obstDist - CollisionDmax) * closestObstacleVec / obstDist;

                        if (float.IsNaN(correction.X) || float.IsNaN(correction.Y))
                        {
                            float noop;
                        }

                        // Voronoi term
                        if (obstDist < VoronoiDmax && VoronoiWeight > 0f)
                        {
                            Vector2 closestVoronoiVec = curr - grid.CellPositionToPoint(grid.GVD.GetNearestVoronoiEdge(grid.PointToCellPosition(curr)));
                            float voroDist = closestVoronoiVec.Length();

                            if (voroDist > 0f)
                            {
                                float alphaplusdo = grid.GVD.AlphaActual + obstDist;
                                float dominusdmax = obstDist - VoronoiDmax;
                                float doplusdv = obstDist + voroDist;
                                float dmaxsquared = VoronoiDmax * VoronoiDmax;
                                float pvdv = (grid.GVD.AlphaActual / alphaplusdo) * (dominusdmax * dominusdmax / dmaxsquared) * (obstDist / (doplusdv * doplusdv));
                                float pvdo = (grid.GVD.AlphaActual / alphaplusdo) * (voroDist / doplusdv) * (dominusdmax / dmaxsquared) * (-dominusdmax / alphaplusdo - dominusdmax / doplusdv + 2);

                                correction -= VoronoiWeight * (pvdo * closestObstacleVec / obstDist + pvdv * closestVoronoiVec / voroDist);

                                if (float.IsNaN(correction.X) || float.IsNaN(correction.Y))
                                {
                                    float noop;
                                }
                            }
                        }
                    }

                    // Smoothing term
                    correction -= SmoothWeight * (currentPath[i - 2].Position - 4 * currentPath[i - 1].Position + 6 * curr - 4 * currentPath[i + 1].Position + currentPath[i + 2].Position);

                    if (float.IsNaN(correction.X) || float.IsNaN(correction.Y))
                    {
                        float noop;
                    }
                    
                    // Curvature term
                    correction -= CurvatureWeight * calcCurvatureTerm(currentPath[i - 1].Position, currentPath[i].Position, currentPath[i + 1].Position);
                    if (float.IsNaN(correction.X) || float.IsNaN(correction.Y))
                    {
                        calcCurvatureTerm(currentPath[i - 1].Position, currentPath[i].Position, currentPath[i + 1].Position);
                    }

                    correction /= CollisionWeight + SmoothWeight + VoronoiWeight + PathWeight + CurvatureWeight;

                    currentPath[i].Position = curr + Alpha * correction;
                    change += correction.Length();
                }

                count++;
                if (Delay > 0)
                    Thread.Sleep(Delay);
            }

            NumIterations = count;
            Change = change;

            ArrayList<Pose> thenewpath = new ArrayList<Pose>();
            thenewpath.AddAll(currentPath);

            if (Anchoring && unsafeIndices == null)
            {
                HashSet<int> newUnsafes = checkPath(thenewpath, grid);
                if (newUnsafes.Count > 0)
                    thenewpath = Smooth(path, grid, newUnsafes);
            }

            foreach (Pose p in thenewpath)
            {
                if (float.IsNaN(p.Position.X) || float.IsNaN(p.Position.Y))
                {
                    float noop;
                }
            }

            return thenewpath;
        }

        public static void Draw(DebugViewXNA debug)
        {
            for (int i = 0; i < currentCount - 1; i++)
            {
                debug.DrawSegment(currentPath[i].Position, currentPath[i + 1].Position, Color.Green, 0.04f);
            }
        }

        private static Vector2 calcCurvatureTerm(Vector2 xim1, Vector2 xi, Vector2 xip1)
        {
            Vector2 dxi = xi - xim1;
            Vector2 dxip1 = xip1 - xi;

            float dphi, ddphi;
            Vector2 p1, p2;
            dphi = (float)Math.Acos(MathHelper.Clamp((dxi.X * dxip1.X + dxi.Y * dxip1.Y) / (dxi.Length() * dxip1.Length()), -1f, 1f));
            float k = dphi / dxi.Length();
            if (k <= KMax)
                return Vector2.Zero;

            ddphi = (float)(-1 / Math.Sqrt(1 - Math.Pow(Math.Cos(dphi), 2)));
            float denom = xi.Length() * xip1.Length();
            p1 = perp(xi, -xip1) / denom;
            p2 = perp(-xip1, xi) / denom;

            float coeff1 = -1 / dxi.Length() * ddphi;
            float coeff2 = dphi / dxi.LengthSquared();

            Vector2 ki, kim1, kip1;
            ki = coeff1 * (-p1 - p2) - coeff2 * Vector2.One;
            kim1 = coeff1 * p2 - coeff2 * -Vector2.One;
            kip1 = coeff1 * p1;

            //return (k - KMax) * (2 * kim1 - 4 * ki + 2 * kip1);
            return (k - KMax) * (0.25f * kim1 + 0.5f * ki + 0.25f * kip1);
        }

        private static Vector2 perp(Vector2 a, Vector2 b) {
            return a - (a.X * b.X + a.Y * b.Y) * b / b.LengthSquared();
        }

        private static ArrayList<Pose> superSample(ArrayList<Pose> path, int num)
        {
            ArrayList<Pose> newPath = new ArrayList<Pose>();

            int count = path.Count;
            for (int i = 0; i < count - 1; i++)
            {
                Vector2 curr = path[i].Position, next = path[i + 1].Position;
                Gear currGear = path[i].Gear;
                for (int n = 0; n < num; n++)
                {
                    newPath.Add(new Pose(Vector2.Lerp(curr, next, (float)n / num), 0f, currGear));
                }
            }

            newPath.Add(path[count - 1]);

            return newPath;
        }

        private static HashSet<int> checkPath(ArrayList<Pose> path, ObstacleGrid grid)
        {
            HashSet<int> unsafeIndices = new HashSet<int>();

            int count = path.Count;

            for (int i = 2; i < count - 2; i++)
            {
                if (path[i - 2].Gear != path[i - 1].Gear) continue;
                if (path[i - 1].Gear != path[i].Gear) continue;
                if (path[i].Gear != path[i + 1].Gear) continue;
                if (path[i + 1].Gear != path[i + 2].Gear) continue;

                Vector2 currPos = path[i].Position;
                Vector2 prevPos = path[i - 1].Position;
                Vector2 displacement = 0.25f * (currPos - prevPos) + 0.75f * (currPos - currPos);
                float pathOrientation = (float)Math.Atan2(displacement.Y, displacement.X);

                if (!grid.IsSafe(new Pose(currPos, pathOrientation), 1.1f))
                {
                    unsafeIndices.Add(i);
                    unsafeIndices.Add(i - 1);
                }
            }

            return unsafeIndices;
        }

        public static string HandleParamCommand(string[] args)
        {
            if (args.Length < 2)
                return GetParams();

            if (args.Length < 3)
                return GetParam(args[1]);

            return UpdateParam(args[1], args[2]);
        }

        public static string GetParam(string param)
        {
            var field = typeof(Smoother).GetField(param);
            if (field == null)
                return "Parameter " + param + " does not exist!";

            return string.Format("{0}: {1}", field.Name, field.GetValue(null));
        }

        public static string UpdateParam(string param, string value)
        {
            var field = typeof(Smoother).GetField(param);
            if (field == null)
                return "Parameter " + param + " does not exist!";

            field.SetValue(null, Convert.ChangeType(value, field.FieldType));
            return "";
        }

        public static string GetParams()
        {
            string str = "Smoother Parameters:\n";
            foreach (var field in typeof(Smoother).GetFields())
                str += "    " + GetParam(field.Name) + "\n";
            return str;
        }
    }
}