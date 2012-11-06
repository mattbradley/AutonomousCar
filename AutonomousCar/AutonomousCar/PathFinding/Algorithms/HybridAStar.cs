using System;
using System.Threading;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using AutonomousCar.Entities;
using AutonomousCar.PathFinding.ReedsShepp;
using C5;

namespace AutonomousCar.PathFinding.Algorithms
{
    /// <summary>
    /// The HybridAStarResults class contains statistics on execution of the Hybrid A* path planner.
    /// </summary>
    public struct HybridAStarResults
    {
        public LinkedList<HybridAStar.Node> Expanded;
        public LinkedList<HybridAStar.Node> Discovered;
        public ArrayList<Pose> Path;
        public TimeSpan HeuristicInitTime;

        public HybridAStarResults(ArrayList<Pose> path, LinkedList<HybridAStar.Node> expanded, LinkedList<HybridAStar.Node> discovered, TimeSpan heuristicInitTime)
        {
            Expanded = expanded;
            Discovered = discovered;
            Path = path;
            HeuristicInitTime = heuristicInitTime;
        }
    }

    /// <summary>
    /// The HybridAStar class implements the Hybrid A* search algorithm. It searches through four-dimensional space (x, y, orientation, gear)
    /// from the start node to the goal node using some heuristic to focus the search direction. This algorithm closely mirrors the standard
    /// A* algorithm; however, each 4D grid cell has an associated 3D vehicle pose. Only one vehicle pose can be associated with a grid cell.
    /// If second node is expanded into the same grid cell, the better node (lower f-value) is kept.
    /// </summary>
    public static class HybridAStar
    {
        private const int numOrientations = 36;
        private const float orientationResolution = MathHelper.TwoPi / numOrientations;

        public const float reverseFactor = 4f;
        public const float switchPenalty = 8f; // meters

        public static float VoronoiFieldFactor = 1.5f;
        public static float SafetyFactor = 1f;
        public static float Epsilon = 1f;
        public static float GridResolution = 1f;

        public static LinkedList<Node> Expanded;
        public static int Delay = 0;

        private static Heuristic heuristic;
        private static Random random = new Random();
        private static Texture2D heuristicBitmap;
        private static bool heuristicUpdated;
        private static Pose goal;

        /// <summary>
        /// The Node class represents a node in the A* search graph. It stores the grid cell, continuous position, f, and g values.
        /// This class can also store Reeds-Shepp information if it's generated as a Reeds-Shepp child path.
        /// </summary>
        public class Node : IComparable<Node>
        {
            public Cell cell;
            public Pose pose;
            public ReedsSheppAction action;
            public ReedsSheppActionSet actionSet;
            public float f;
            public float g;
            public Node from;
            public int rsIndex;

            public Node(Cell cell, ReedsSheppAction action, Pose pose, float g, float f, Node from)
            {
                this.cell = cell;
                this.pose = pose;
                this.action = action;
                this.actionSet = null;
                this.g = g;
                this.f = f;
                this.from = from;

                rsIndex = -1;
            }

            public Node(Cell cell, ReedsSheppActionSet actionSet, Pose pose, float g, float f, Node from)
            {
                this.cell = cell;
                this.pose = pose;
                this.action = null;
                this.actionSet = actionSet;
                this.g = g;
                this.f = f;
                this.from = from;

                rsIndex = -1;
            }

            public int CompareTo(Node other)
            {
                return f.CompareTo(other.f);
            }
        }

        private struct CellState
        {
            public Pose continuousPose;
            public float f;
            public IPriorityQueueHandle<Node> handle;

            public CellState(Pose continuous, float f, IPriorityQueueHandle<Node> handle)
            {
                this.continuousPose = continuous;
                this.f = f;
                this.handle = handle;
            }
        }

        public struct Cell
        {
            public int c, r, o, rev;

            public Cell(int column, int row, int orientation, int reverse)
            {
                this.c = column;
                this.r = row;
                this.o = orientation;
                this.rev = reverse;
            }

            public static Cell None = new Cell(-1, -1, 0, 0);
        }

        private static ObstacleGrid grid;

        public static void Reset()
        {
            Expanded = new LinkedList<Node>();
        }

        public static HybridAStarResults FindPath(ObstacleGrid aGrid, Pose start, Pose aGoal)
        {
            grid = aGrid;
            goal = aGoal;
            heuristicUpdated = false;
            heuristicBitmap = null;
            IntervalHeap<Node> open = new IntervalHeap<Node>();
            //heuristic = new EuclideanHeuristic();
            //heuristic = new ObstacleRelaxed(grid);
            heuristic = new NonholonomicRelaxed(grid);
            //heuristic = new CombinedHeuristic(grid);

            DateTime now = DateTime.Now;
            heuristic.Update(goal);
            heuristicUpdated = true;
            TimeSpan heuristicInitTime = DateTime.Now - now;

            Expanded = new LinkedList<Node>();
            LinkedList<Node> discovered = new LinkedList<Node>();

            int numCols = (int)Math.Ceiling(grid.NumColumns * grid.Resolution / GridResolution);
            int numRows = (int)Math.Ceiling(grid.NumRows * grid.Resolution / GridResolution);
            CellState?[, , ,] cells = new CellState?[numCols, numRows, numOrientations, 2];

            Cell startCell = poseToCell(start, 0);
            IPriorityQueueHandle<Node> startHandle = null;
            float heuristicValue = heuristic.GetHeuristicValue(start, goal);
            Node startNode = new Node(startCell, new ReedsSheppAction(Steer.Straight, Gear.Forward, 0f), start, 0, Epsilon * heuristicValue, null);
            open.Add(ref startHandle, startNode);
            discovered.Add(startNode);
            cells[startCell.c, startCell.r, startCell.o, startCell.rev] = new CellState(start, Epsilon * heuristicValue, startHandle);

            while (!open.IsEmpty)
            {
                Node n = open.DeleteMin();

                lock (Expanded)
                {
                    Expanded.Add(n);
                }
                CellState? c = cells[n.cell.c, n.cell.r, n.cell.o, n.cell.rev];

                if (c.Value.continuousPose == goal)
                    return new HybridAStarResults(reconstructPath(n), Expanded, discovered, heuristicInitTime);

                float dt = GridResolution / VehicleModel.SlowVelocity;
                //GridCell nodeCell = grid.PointToCellPosition(c.Value.continuousPose.Position);
                //float dt = MathHelper.Max(GridResolution, 0.5f * (grid.GVD.GetObstacleDistance(nodeCell) + grid.GVD.GetVoronoiDistance(nodeCell)));
                //dt /= VehicleModel.SlowVelocity;

                for (int g = 0; g < VehicleModel.NumGears; g++)
                {
                    Gear gear = (Gear)g;

                    foreach (Node child in getChildren(c.Value.continuousPose, gear, dt, goal))
                    {
                        int rev = gear == Gear.Forward ? 0 : 1;
                        float tentativeg;

                        if (child.actionSet == null)
                            tentativeg = n.g + pathCost(n.pose, n.cell.rev, child.pose, rev, dt);
                        else
                            tentativeg = n.g + child.actionSet.CalculateCost(VehicleModel.TurnRadius, reverseFactor, switchPenalty);

                        float tentativef = tentativeg + Epsilon * heuristic.GetHeuristicValue(child.pose, goal);

                        child.cell = poseToCell(child.pose, rev);
                        CellState? currentCell = cells[child.cell.c, child.cell.r, child.cell.o, child.cell.rev];

                        if (!currentCell.HasValue)
                        {
                            IPriorityQueueHandle<Node> childHandle = null;
                            child.g = tentativeg;
                            child.f = tentativef;
                            child.from = n;

                            open.Add(ref childHandle, child);
                            discovered.Add(child);
                            cells[child.cell.c, child.cell.r, child.cell.o, child.cell.rev] = new CellState(child.pose, tentativef, childHandle);
                        }
                        else if (tentativef < currentCell.Value.f)
                        {
                            IPriorityQueueHandle<Node> currentHandle = currentCell.Value.handle;
                            child.g = tentativeg;
                            child.f = tentativef;
                            child.from = n;

                            Node temp;
                            if (open.Find(currentHandle, out temp))
                                open[currentHandle] = child;
                            else
                                open.Add(ref currentHandle, child);

                            discovered.Add(child);
                            cells[child.cell.c, child.cell.r, child.cell.o, child.cell.rev] = new CellState(child.pose, tentativef, currentHandle);
                        }
                    }
                }

                if (Delay > 0)
                    Thread.Sleep(Delay);
            }

            return new HybridAStarResults(new ArrayList<Pose>(), Expanded, discovered, heuristicInitTime);
        }

        private static LinkedList<Node> getChildren(Pose pose, Gear gear, float dt, Pose goal)
        {
            LinkedList<Node> children = new LinkedList<Node>();

            for (int steer = 0; steer < VehicleModel.NumSteers; steer++)
            {
                float length;
                Pose child = VehicleModel.NextPose(pose, (Steer)steer, gear, dt, out length);
                child.Gear = gear;
                if (grid.IsPointInGrid(child.Position) && grid.IsSafe(child, SafetyFactor))
                    children.Add(new Node(Cell.None, new ReedsSheppAction((Steer)steer, (Gear)gear, length), child, 0, 0, null));
            }

            // Add a Reeds-Shepp curve as a child every now and then
            float cost = heuristic.GetHeuristicValue(pose, goal);

            // Made the fall off rate quadratic because Reeds-Shepp curves are expsensive!
            if (cost < 10f || random.NextDouble() < 10f / (cost * cost))
            {
                Node reedsShepp = getReedsSheppChild(pose, goal);
                if (reedsShepp != null)
                    children.Add(reedsShepp);
            }

            return children;
        }

        private static Node getReedsSheppChild(Pose pose, Pose goal)
        {
            ReedsSheppActionSet actions = ReedsSheppSolver.Solve(pose, goal, VehicleModel.TurnRadius);
            bool safe = true;
            foreach (Pose s in ReedsSheppDriver.Discretize(pose, actions, VehicleModel.TurnRadius, GridResolution))
            {
                if (!grid.IsPointInGrid(s.Position) || !grid.IsSafe(s, SafetyFactor))
                {
                    safe = false;
                    break;
                }
            }

            if (safe)
                return new Node(Cell.None, actions, goal, 0, 0, null);

            return null;
        }

        private static float pathCost(Pose from, int fromRev, Pose to, int toRev, float dt)
        {
            float dist = dt * VehicleModel.SlowVelocity;
            float revCost = 0;

            if (toRev == 1)
                revCost = dist * reverseFactor;
            if (fromRev != toRev)
                revCost += switchPenalty;

            return revCost + dist + dist * VoronoiFieldFactor * grid.GVD.GetPathCost(grid.PointToCellPosition(to.Position));
        }

        private static ArrayList<Pose> reconstructPath(Node node)
        {
            LinkedList<Pose> path = new LinkedList<Pose>();
            Node rsNode = null;
            int subpathCount = 0;

            while (node != null)
            {
                if (node.action != null)
                {
                    path.InsertFirst(node.pose);
                }
                else
                {
                    ArrayList<Pose> subpath = ReedsSheppDriver.Discretize(node.from.pose, node.actionSet, VehicleModel.TurnRadius, GridResolution);
                    subpathCount = subpath.Count;
                    for (int i = subpathCount - 1; i >= 1; i--)
                        path.InsertFirst(subpath[i]);

                    rsNode = node;
                }
                node = node.from;
            }

            ArrayList<Pose> pathArray = new ArrayList<Pose>();
            pathArray.AddAll(path);

            if (rsNode != null)
                rsNode.rsIndex = pathArray.Count - subpathCount;

            // Move every node's gear to it's parent node
            int count = pathArray.Count;
            Gear nextGear = pathArray[count - 1].Gear;
            for (int i = count - 2; i >= 0; i--)
            {
                Pose curr = pathArray[i];
                Gear currGear = curr.Gear;
                curr.Gear = nextGear;
                pathArray[i] = curr;
                nextGear = currGear;
            }

            return pathArray;
        }

        private static Cell poseToCell(Pose pose, int rev)
        {
            Vector2 pos = pose.Position - grid.Origin;
            int c = (int)Math.Floor(pos.X / GridResolution);
            int r = (int)Math.Floor(pos.Y / GridResolution);

            float theta = pose.Orientation;
            while (theta < 0) theta += MathHelper.TwoPi;
            while (theta >= MathHelper.TwoPi) theta -= MathHelper.TwoPi;

            int o = (int)Math.Floor(theta / orientationResolution);

            return new Cell(c, r, o, rev);
        }

        public static void Draw(SpriteBatch spritebatch)
        {
            if (!heuristicUpdated)
                return;

            if (heuristicBitmap == null)
            {
                float max = float.MinValue;
                heuristicBitmap = new Texture2D(spritebatch.GraphicsDevice, grid.NumColumns, grid.NumRows);
                float[] values = new float[grid.NumColumns * grid.NumRows];

                for (int c = 0; c < grid.NumColumns; c++)
                    for (int r = 0; r < grid.NumRows; r++)
                    {
                        float val = heuristic.GetHeuristicValue(new Pose(grid.CellPositionToPoint(new GridCell(c, r)), MathHelper.PiOver2), goal);
                        values[c + r * grid.NumColumns] = val;
                        if (val > max && !float.IsInfinity(val))
                            max = val;
                    }

                Color[] colors = new Color[grid.NumColumns * grid.NumRows];
                for (int c = 0; c < grid.NumColumns; c++)
                    for (int r = 0; r < grid.NumRows; r++)
                    {
                        float val = 1f - values[c + r * grid.NumColumns] / max;
                        colors[c + r * grid.NumColumns] = new Color(val, val, val);
                    }

                heuristicBitmap.SetData<Color>(colors);
            }

            spritebatch.Draw(heuristicBitmap, grid.Origin, null, Color.White, 0, Vector2.Zero, grid.Resolution, SpriteEffects.None, 0f);
        }
    }
}