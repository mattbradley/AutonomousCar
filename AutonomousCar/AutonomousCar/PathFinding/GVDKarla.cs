using System;
using C5;

namespace AutonomousCar.PathFinding
{
    /// <summary>
    /// This class implements a grid-based generalized voronoi diagram based on the dynamic brushfire algorithm described in Incremental Reconstruction of Generalized Voronoi Diagrams on Grids.
    /// http://www.ri.cmu.edu/pub_files/pub4/kalra_nidhi_2006_2/kalra_nidhi_2006_2.pdf
    /// 
    /// This implementation is replaced by an improved version by Lau, et al. This class is no longer used.
    /// </summary>
    public class GVDKarla
    {
        public bool[,] VoronoiEdges { get { return voro; } }

        private ObstacleGrid grid;
        private LinkedList<GridCell> ties;
        private IntervalHeap<GridCellValue> open;
        private float[,] dist, distNew;
        private GridCell[,] parent, tie;
        private int[,] obst;
        private HashSet<int> valid;
        private bool[,] voro;

        private float sqrt2 = (float)Math.Sqrt(2);

        public GVDKarla(ObstacleGrid grid)
        {
            this.grid = grid;

            open = new IntervalHeap<GridCellValue>();
            ties = new LinkedList<GridCell>();
            dist = new float[grid.NumColumns, grid.NumRows];
            distNew = new float[grid.NumColumns, grid.NumRows];
            parent = new GridCell[grid.NumColumns, grid.NumRows];
            tie = new GridCell[grid.NumColumns, grid.NumRows];
            obst = new int[grid.NumColumns, grid.NumRows];
            valid = new HashSet<int>();
            voro = new bool[grid.NumColumns, grid.NumRows];

            for (int c = grid.NumColumns - 1; c >= 0; c--)
                for (int r = grid.NumRows - 1; r >= 0; r--)
                {
                    dist[c, r] = float.PositiveInfinity;
                    distNew[c, r] = float.PositiveInfinity;
                    parent[c, r] = GridCell.Unknown;
                    tie[c, r] = GridCell.Unknown;
                    obst[c, r] = -1;
                    voro[c, r] = false;
                }
        }

        public void SetObstacle(GridCell cell, int obstacleId)
        {
            distNew[cell.C, cell.R] = 0f;
            obst[cell.C, cell.R] = obstacleId;
            parent[cell.C, cell.R] = cell;
            valid.Add(obstacleId);

            open.Add(new GridCellValue(cell, 0f));
        }

        public void UnsetObstacle(GridCell cell)
        {
            distNew[cell.C, cell.R] = float.PositiveInfinity;
            obst[cell.C, cell.R] = -1;

            open.Add(new GridCellValue(cell, dist[cell.C, cell.R]));
        }

        public void InvalidateObstacle(int obstacleId)
        {
            if (valid.Contains(obstacleId))
                valid.Remove(obstacleId);
        }

        public void UpdateDistanceMap()
        {
            while (!open.IsEmpty)
            {
                GridCell cell = open.DeleteMin().Position;

                if (distNew[cell.C, cell.R] < dist[cell.C, cell.R])
                {
                    dist[cell.C, cell.R] = distNew[cell.C, cell.R];
                    processLower(cell);
                    considerForGVD(cell);
                }
                else
                {
                    dist[cell.C, cell.R] = float.PositiveInfinity;
                    float dN = distNew[cell.C, cell.R];
                    if (dN != float.PositiveInfinity)
                        open.Add(new GridCellValue(cell, dN));
                    processRaise(cell);
                }
            }

            constructGVD();
        }

        public float GetDistance(GridCell cell) { return GetDistance(cell.C, cell.R); }
        public float GetDistance(int c, int r) { return dist[c, r]; }

        private void considerForGVD(GridCell cell)
        {
            foreach (GridCell adj in grid.Get8Neighbors(cell))
            {
                if (obst[cell.C, cell.R] != obst[adj.C, adj.R])
                {
                    tie[cell.C, cell.R] = adj;
                    ties.Add(cell);
                }
            }
        }

        private void constructGVD()
        {
            foreach (GridCell cell in ties)
            {
                foreach (GridCell adj in grid.Get8Neighbors(cell))
                {
                    if (obst[cell.C, cell.R] != obst[adj.C, adj.R])
                    {
                        voro[cell.C, cell.R] = true;
                        voro[adj.C, adj.R] = true;
                        tie[cell.C, cell.R] = adj;
                        tie[adj.C, adj.R] = cell;
                    }
                    else
                    {
                        voro[cell.C, cell.R] = false;
                        voro[adj.C, adj.R] = false;
                        tie[cell.C, cell.R] = GridCell.Unknown;
                        tie[adj.C, adj.R] = GridCell.Unknown;
                    }
                }
            }

            ties.Clear();
        }

        private void processLower(GridCell cell)
        {
            foreach (GridCell adj in grid.Get8Neighbors(cell))
            {
                if (tie[adj.C, adj.R] == cell)
                    ties.Add(adj);

                float dNadj = distNew[adj.C, adj.R];
                float dNcell = distNew[cell.C, cell.R];
                if (dNadj > dNcell)
                {
                    float d = adjDistance(adj, cell) + dNcell;
                    if (d < dNadj)
                    {
                        distNew[adj.C, adj.R] = d;
                        parent[adj.C, adj.R] = cell;
                        obst[adj.C, adj.R] = obst[cell.C, cell.R];
                        open.Add(new GridCellValue(adj, d));
                    }
                }
            }
        }

        private void processRaise(GridCell cell)
        {
            foreach (GridCell adj in grid.Get8Neighbors(cell))
            {
                if (tie[adj.C, adj.R] == cell)
                    ties.Add(adj);

                if (parent[adj.C, adj.R] == cell)
                {
                    float distOld = distNew[adj.C, adj.R];
                    distNew[adj.C, adj.R] = float.PositiveInfinity;
                    int obstOld = obst[adj.C, adj.R];

                    foreach (GridCell a in grid.Get8Neighbors(adj))
                    {
                        int aObst = obst[a.C, a.R];
                        if (aObst != -1 && valid.Contains(aObst))
                        {
                            float d = adjDistance(adj, a) + distNew[a.C, a.R];
                            if (d < distNew[adj.C, adj.R])
                            {
                                distNew[adj.C, adj.R] = d;
                                parent[adj.C, adj.R] = a;
                                obst[adj.C, adj.R] = aObst;
                            }
                        }
                    }

                    if (distNew[adj.C, adj.R] != distOld || obst[adj.C, adj.R] != obstOld)
                        open.Add(new GridCellValue(adj, Math.Min(distNew[adj.C, adj.R], dist[adj.C, adj.R])));
                }
            }
        }

        private float adjDistance(GridCell a, GridCell b)
        {
            if (a.C == b.C || a.R == b.R)
                return 1;
            else
                return sqrt2;
        }
    }
}
