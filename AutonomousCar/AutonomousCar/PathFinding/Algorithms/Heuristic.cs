using System;
using System.IO;
using System.Runtime.Serialization;
using System.Runtime.Serialization.Formatters.Binary;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using AutonomousCar.PathFinding.ReedsShepp;
using AutonomousCar.Entities;
using C5;

namespace AutonomousCar.PathFinding.Algorithms
{
    /// <summary>
    /// The Heuristic abstract base class.
    /// </summary>
    public abstract class Heuristic
    {
        public virtual float GetHeuristicValue(Pose pose, Pose goal) { return 0f; }
        public virtual void Update(Pose goal) { }
    }

    [Serializable()]
    public struct NonholonomiHeuristicInfo
    {
        public int NumCells;
        public int NumOrientations;
        public float OrientationSize;
        public float CellSize;
        public float[, ,] Heuristic;
        public float MaxHeuristic;
    }

    /// <summary>
    /// The EuclideanHeuristic class returns the distance from pose to goal as the heuristic value.
    /// </summary>
    public class EuclideanHeuristic : Heuristic
    {
        public override float GetHeuristicValue(Pose pose, Pose goal)
        {
            return (goal.Position - pose.Position).Length();
        }
    }


    /// <summary>
    /// The ObstacleRelaxed class implements the non-holonomic-without-obstacles heuristic. This heuristic is pre-computed by the HeuristicCalculator project
    /// and loaded in at the start of the path planning routine. This heuristic value is the length of the optimal Reeds-Shepp path from the pose to the goal
    /// without taking any obstacles into account.
    /// </summary>
    public class ObstacleRelaxed : Heuristic
    {
        private ObstacleGrid grid;
        private const int numOrientations = 72;
        private const float orientationResolution = MathHelper.TwoPi / numOrientations;

        private NonholonomiHeuristicInfo info;
        private float offset;

        public ObstacleRelaxed(ObstacleGrid grid)
        {
            this.grid = grid;
        }

        public override void Update(Pose goal)
        {
            Stream stream = File.OpenRead(@"../../../rsheurnorev.dat");
            info = (NonholonomiHeuristicInfo)(new BinaryFormatter()).Deserialize(stream);
            stream.Close();

            offset = (float)Math.Floor(info.NumCells / 2f) * info.CellSize;
        }

        public float GetHeuristicValue(Pose pose)
        {
            return GetHeuristicValue(pose, new Pose());
        }

        public override float GetHeuristicValue(Pose pose, Pose goal)
        {
            Vector2 pos = Vector2.Transform(pose.Position, Matrix.CreateTranslation(new Vector3(-goal.Position, 0f)) * Matrix.CreateRotationZ(-goal.Orientation));
            float orientation = pose.Orientation - goal.Orientation;
            while (orientation < 0) orientation += MathHelper.TwoPi;
            while (orientation >= MathHelper.TwoPi) orientation -= MathHelper.TwoPi;

            int c = (int)Math.Round((pos.X + offset) / info.CellSize);
            int r = (int)Math.Round((pos.Y + offset) / info.CellSize);
            int o = (int)Math.Round(orientation / info.OrientationSize);
            if (o == info.NumOrientations)
                o = 0;

            if (c < 0 || r < 0 || c >= info.NumCells || r >= info.NumCells)
                return (pose.Position - goal.Position).Length();

            return info.Heuristic[c, r, o];
        }
    }

    /// <summary>
    /// The NonholonomicRelaxed class implements the holonomic-with-obstacles heuristic. This heuristic is calculated using Djikstra's algorithm on the
    /// obstacle grid. This heuristic value is the 8-neighbor path length from the pose cell to the goal cell. It considers obstacles but ignores the
    /// turning radius constraint of the vehicle.
    /// </summary>
    public class NonholonomicRelaxed : Heuristic
    {
        private ObstacleGrid grid;
        private float[,] heuristic;
        private float unit;

        private const float sqrt2 = 1.4142135623730950488016887242097f;
        private const float discount = 0.92621f;

        public NonholonomicRelaxed(ObstacleGrid grid)
        {
            this.grid = grid;
            heuristic = new float[grid.NumColumns, grid.NumRows];
            unit = grid.Resolution * discount; // factor to discount the suboptimality of 8-neighbor paths
        }

        public override void Update(Pose goal)
        {
            bool[,] expanded = new bool[grid.NumColumns, grid.NumRows];
            LinkedList<GridCell>[,] memoized = new LinkedList<GridCell>[grid.NumColumns, grid.NumRows];
            GridCell startCell = grid.PointToCellPosition(goal.Position);
            IntervalHeap<GridCellValue> open = new IntervalHeap<GridCellValue>();

            for (int c = 0; c < grid.NumColumns; c++)
                for (int r = 0; r < grid.NumRows; r++)
                {
                    heuristic[c, r] = float.PositiveInfinity;
                    expanded[c, r] = false;
                }

            heuristic[startCell.C, startCell.R] = 0f;
            open.Add(new GridCellValue(startCell, 0f));

            while (!open.IsEmpty)
            {
                GridCell cell = open.DeleteMin().Position;
                expanded[cell.C, cell.R] = true;

                LinkedList<GridCell> neighbors = memoized[cell.C, cell.R];
                if (neighbors == null)
                {
                    neighbors = grid.Get8Neighbors(cell);
                    memoized[cell.C, cell.R] = neighbors;
                }

                foreach (GridCell n in neighbors)
                {
                    if (expanded[n.C, n.R])
                        continue;

                    if (grid.OccupiedCells[n.C, n.R] > 0)
                        continue;

                    if (grid.GVD.GetObstacleDistance(n) <= Car.HALF_CAR_WIDTH)
                        continue;

                    float dist = cell.C == n.C || cell.R == n.R ? 1f : sqrt2;
                    dist += heuristic[cell.C, cell.R];

                    if (dist < heuristic[n.C, n.R])
                    {
                        heuristic[n.C, n.R] = dist;
                        open.Add(new GridCellValue(n, dist));
                    }
                }
            }
        }

        public override float GetHeuristicValue(Pose pose, Pose goal)
        {
            GridCell cell = grid.PointToCellPosition(pose.Position);
            return MathHelper.Max(unit * heuristic[cell.C, cell.R] - grid.DiagonalResolution, (goal.Position - pose.Position).Length());
        }
    }

    /// <summary>
    /// The CombinedHeuristic class combines the non-holonomic-without-obstacles and the holonomic-with-obstacles heuristics by 
    /// returning the maximum between the two as the heuristic value.
    /// </summary>
    public class CombinedHeuristic : Heuristic
    {
        private Heuristic one, two;

        public CombinedHeuristic(ObstacleGrid grid)
        {
            one = new NonholonomicRelaxed(grid);
            two = new ObstacleRelaxed(grid);
        }

        public override void Update(Pose goal)
        {
            one.Update(goal);
            two.Update(goal);
        }

        public override float GetHeuristicValue(Pose pose, Pose goal)
        {
            return MathHelper.Max(one.GetHeuristicValue(pose, goal), two.GetHeuristicValue(pose, goal));
        }
    }
}
