using System;
using AutonomousCar.PathFinding;
using AutonomousCar.PathFinding.ReedsShepp;
using Microsoft.Xna.Framework;

namespace AutonomousCar.PathFinding.Algorithms
{
    /// <summary>
    /// The HeuristicCalculator class is used by the HeuristicCalculator project to precompute the non-holonomic-without-obstacles heuristic.
    /// This is done by finding the length of the optimal Reeds-Shepp path to the goal from each cell in some neighborhood of the goal cell.
    /// </summary>
    public static class HeuristicCalculator
    {
        private static int numCells;
        private static float orientationSize;
        private static float offset;

        public delegate void CellCalculatedEvent(object sender, CalculatedArgs e);
        public static event CellCalculatedEvent CellCalculated;

        public static NonholonomiHeuristicInfo Calculate(float neighborhoodSize, float cellSize, int numOrientations)
        {
            orientationSize = MathHelper.TwoPi / numOrientations;
            numCells = (int)Math.Ceiling(neighborhoodSize / cellSize);
            if (numCells % 2 == 0)
                numCells++;

            offset = (float)Math.Floor(numCells / 2f) * cellSize;
            Pose goal = new Pose(0f, 0f, 0f);

            float[, ,] heur = new float[numCells, numCells, numOrientations];
            int totalCells = numCells * numCells * numOrientations;
            int count = 0;
            float maxHeuristic = float.MinValue;

            for (int c = 0; c < numCells; c++)
                for (int r = 0; r < numCells; r++)
                    for (int o = 0; o < numOrientations; o++)
                    {
                        Pose pose = new Pose(c * cellSize - offset, r * cellSize - offset, o * orientationSize);
                        ReedsSheppActionSet actions = ReedsSheppSolver.Solve(pose, goal, VehicleModel.TurnRadius);
                        heur[c, r, o] = actions.CalculateCost(VehicleModel.TurnRadius, 1f, 0f);
                        if (actions.Length > maxHeuristic)
                            maxHeuristic = actions.Length;
                        count++;

                        if (CellCalculated != null)
                            CellCalculated(null, new CalculatedArgs()
                            {
                                C = c,
                                R = r,
                                O = o,
                                Progress = (float)count / totalCells
                            });
                    }

            return new NonholonomiHeuristicInfo()
            {
                CellSize = cellSize,
                NumCells = numCells,
                OrientationSize = orientationSize,
                NumOrientations = numOrientations,
                Heuristic = heur,
                MaxHeuristic = maxHeuristic
            };
        }
    }

    public class CalculatedArgs : EventArgs
    {
        public int C;
        public int R;
        public int O;
        public float Progress;
    }
}
