using System;
using C5;

namespace AutonomousCar.PathFinding
{
    // This class implements an updated grid-based generalized voronoi diagram from
    // Improved Updating of Euclidean Distance Maps and Voronoi Diagrams
    // http://www.first-mm.eu/files/lau10iros.pdf

    /// <summary>
    /// This class implements an updated grid-based generalized voronoi diagram from Improved Updating of Euclidean Distance Maps and Voronoi Diagrams.
    /// http://www.first-mm.eu/files/lau10iros.pdf
    /// 
    /// This class replaced GVDKarla and is currently used to generate the distance field, GVD, and Voronoi field.
    /// </summary>
	public class GVDLau
	{
        public const float AlphaDefault = 20f; // in meters
        public const float DMaxDefault = 30f; // in meters

        public float DmaxActual { get; private set; }
        public float AlphaActual { get; private set; }
        public bool[,] VoronoiEdges { get { return voro; } }

        private ObstacleGrid grid;

        // Obstacle distance map
        private int[,] dist;
        private float[,] distActual;
        private GridCell[,] obst;
        private bool[,] toRaise;
        private bool[,] toProcess;
        private bool[,] voro;
        private IntervalHeap<GridCellValue> voroQ;
        private IntervalHeap<GridCellValue> open;
        private int[,] comp;

        // Voronoi distance map
        private int[,] voroDist;
        private float[,] voroDistActual;
        private GridCell[,] nearestVoro;
        private bool[,] voroToRaise;
        private bool[,] voroToProcess;
        private IntervalHeap<GridCellValue> voroOpen;

        private float[,] pathCost;

        public GVDLau(ObstacleGrid grid)
        {
            this.grid = grid;

            // Obstacle distance map
            dist = new int[grid.NumColumns, grid.NumRows];
            distActual = new float[grid.NumColumns, grid.NumRows];
            obst = new GridCell[grid.NumColumns, grid.NumRows];
            toRaise = new bool[grid.NumColumns, grid.NumRows];
            toProcess = new bool[grid.NumColumns, grid.NumRows];
            voro = new bool[grid.NumColumns, grid.NumRows];
            voroQ = new IntervalHeap<GridCellValue>();
            open = new IntervalHeap<GridCellValue>();
            comp = new int[grid.NumColumns, grid.NumRows];

            // Voronoi distance map
            voroDist = new int[grid.NumColumns, grid.NumRows];
            voroDistActual = new float[grid.NumColumns, grid.NumRows];
            nearestVoro = new GridCell[grid.NumColumns, grid.NumRows];
            voroToRaise = new bool[grid.NumColumns, grid.NumRows];
            voroToProcess = new bool[grid.NumColumns, grid.NumRows];
            voroOpen = new IntervalHeap<GridCellValue>();

            pathCost = new float[grid.NumColumns, grid.NumRows];
            DmaxActual = DMaxDefault / grid.Resolution;
            AlphaActual = AlphaDefault / grid.Resolution;

            for (int c = grid.NumColumns - 1; c >= 0; c--)
                for (int r = grid.NumRows - 1; r >= 0; r--)
                {
                    // Obstacle distance map
                    dist[c, r] = int.MaxValue;
                    distActual[c, r] = float.PositiveInfinity;
                    obst[c, r] = GridCell.Unknown;
                    toRaise[c, r] = false;
                    toProcess[c, r] = false;
                    voro[c, r] = true;
                    comp[c, r] = -1;

                    // Voronoi distance map
                    voroDist[c, r] = int.MaxValue;
                    voroDistActual[c, r] = float.PositiveInfinity;
                    nearestVoro[c, r] = GridCell.Unknown;
                    voroToRaise[c, r] = false;
                    voroToProcess[c, r] = false;

                    pathCost[c, r] = 0f;
                }
        }

        public GridCell GetNearestObstacle(GridCell cell)
        {
            return GetNearestObstacle(cell.C, cell.R);
        }

        public GridCell GetNearestObstacle(int c, int r)
        {
            return obst[c, r];
        }

        public GridCell GetNearestVoronoiEdge(GridCell cell)
        {
            return GetNearestVoronoiEdge(cell.C, cell.R);
        }

        public GridCell GetNearestVoronoiEdge(int c, int r)
        {
            return nearestVoro[c, r];
        }


        public float GetObstacleDistance(GridCell cell)
        {
            return GetObstacleDistance(cell.C, cell.R);
        }

        public float GetObstacleDistance(int c, int r)
        {
            return distActual[c, r] * grid.Resolution;
        }

        public float GetVoronoiDistance(GridCell cell)
        {
            return GetVoronoiDistance(cell.C, cell.R);
        }

        public float GetVoronoiDistance(int c, int r)
        {
            return voroDistActual[c, r] * grid.Resolution;
        }

        public float GetPathCost(GridCell cell)
        {
            return GetPathCost(cell.C, cell.R);
        }

        public float GetPathCost(int c, int r)
        {
            return pathCost[c, r];
        }

        public void SetObstacle(GridCell s, int obstId)
        {
            obst[s.C, s.R] = s;
            comp[s.C, s.R] = obstId;
            dist[s.C, s.R] = 0;
            distActual[s.C, s.R] = 0f;
            open.Add(new GridCellValue(s, 0));
            toProcess[s.C, s.R] = true;
        }

        public void UnsetObstacle(GridCell s)
        {
            dist[s.C, s.R] = int.MaxValue;
            distActual[s.C, s.R] = float.PositiveInfinity;
            obst[s.C, s.R] = GridCell.Unknown;
            comp[s.C, s.R] = -1;
            toRaise[s.C, s.R] = true;

            // Typo in line 6 of algorithm 1 in the paper listed above.
            // It should be:
            //     insert(open, s, infinity)
            // instead of:
            //     insert(open, s, 0)
            open.Add(new GridCellValue(s, int.MaxValue));
            toProcess[s.C, s.R] = true;
        }

        public void Update()
        {
            updateDistanceMap();
            updateVoroMap();
            updatePathCostMap();
        }

        private void updateDistanceMap()
        {
            while (!open.IsEmpty)
            {
                GridCell s = open.DeleteMin().Position;
                if (!toProcess[s.C, s.R]) continue;

                if (toRaise[s.C, s.R])
                {
                    // raise(s)
                    foreach (GridCell n in grid.Get8Neighbors(s))
                    {
                        if (obst[n.C, n.R] != GridCell.Unknown && !toRaise[n.C, n.R])
                        {
                            if (!isOcc(obst[n.C, n.R]))
                            {
                                dist[n.C, n.R] = int.MaxValue;
                                distActual[n.C, n.R] = float.PositiveInfinity;
                                obst[n.C, n.R] = GridCell.Unknown;
                                toRaise[n.C, n.R] = true;
                            }

                            open.Add(new GridCellValue(n, dist[n.C, n.R]));
                            toProcess[n.C, n.R] = true;
                        }
                    }

                    toRaise[s.C, s.R] = false;
                }
                else if (isOcc(obst[s.C, s.R]))
                {
                    voro[s.C, s.R] = false;
                    unsetVoro(s);
                    toProcess[s.C, s.R] = false;

                    // lower(s)
                    foreach (GridCell n in grid.Get8Neighbors(s))
                    {
                        if (!toRaise[n.C, n.R])
                        {
                            int d = distanceSquared(obst[s.C, s.R], n);
                            if (d < dist[n.C, n.R])
                            {
                                dist[n.C, n.R] = d;
                                distActual[n.C, n.R] = (float)Math.Sqrt(d);
                                obst[n.C, n.R] = obst[s.C, s.R];
                                open.Add(new GridCellValue(n, d));
                                toProcess[n.C, n.R] = true;
                            }
                            else
                            {
                                chkVoro(s, n);
                            }
                        }
                    }
                }
            }
        }

        private void setVoro(GridCell s)
        {
            nearestVoro[s.C, s.R] = s;
            voroDist[s.C, s.R] = 0;
            voroDistActual[s.C, s.R] = 0f;
            voroOpen.Add(new GridCellValue(s, 0));
            voroToProcess[s.C, s.R] = true;
        }

        private void unsetVoro(GridCell s)
        {
            voroDist[s.C, s.R] = int.MaxValue;
            voroDistActual[s.C, s.R] = float.PositiveInfinity;
            nearestVoro[s.C, s.R] = GridCell.Unknown;
            voroToRaise[s.C, s.R] = true;
            voroOpen.Add(new GridCellValue(s, int.MaxValue));
            voroToProcess[s.C, s.R] = true;
        }

        private void updateVoroMap()
        {
            while (!voroOpen.IsEmpty)
            {
                GridCell s = voroOpen.DeleteMin().Position;
                if (voroToProcess[s.C, s.R])
                {

                    if (voroToRaise[s.C, s.R])
                    {
                        // raise(s)
                        foreach (GridCell n in grid.Get8Neighbors(s))
                        {
                            if (nearestVoro[n.C, n.R] != GridCell.Unknown && !voroToRaise[n.C, n.R])
                            {
                                if (!voroIsOcc(nearestVoro[n.C, n.R]))
                                {
                                    voroDist[n.C, n.R] = int.MaxValue;
                                    voroDistActual[n.C, n.R] = float.PositiveInfinity;
                                    nearestVoro[n.C, n.R] = GridCell.Unknown;
                                    voroToRaise[n.C, n.R] = true;
                                }

                                voroOpen.Add(new GridCellValue(n, voroDist[n.C, n.R]));
                                voroToProcess[n.C, n.R] = true;
                            }
                        }

                        voroToRaise[s.C, s.R] = false;
                    }
                    else if (voroIsOcc(nearestVoro[s.C, s.R]))
                    {
                        voroToProcess[s.C, s.R] = false;

                        // lower(s)
                        foreach (GridCell n in grid.Get8Neighbors(s))
                        {
                            if (!voroToRaise[n.C, n.R])
                            {
                                int d = distanceSquared(nearestVoro[s.C, s.R], n);
                                if (d < voroDist[n.C, n.R])
                                {
                                    voroDist[n.C, n.R] = d;
                                    voroDistActual[n.C, n.R] = (float)Math.Sqrt(d);
                                    nearestVoro[n.C, n.R] = nearestVoro[s.C, s.R];
                                    voroOpen.Add(new GridCellValue(n, d));
                                    voroToProcess[n.C, n.R] = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        public void UpdatePathCostParameters(float alpha, float dmax)
        {
            AlphaActual = alpha / grid.Resolution;
            DmaxActual = dmax / grid.Resolution;
            updatePathCostMap();
        }

        private void updatePathCostMap()
        {
            for (int c = 0; c < grid.NumColumns; c++)
                for (int r = 0; r < grid.NumRows; r++)
                    if (distActual[c, r] >= DmaxActual || float.IsInfinity(voroDistActual[c, r]))
                        pathCost[c, r] = 0f;
                    else
                        pathCost[c, r] = (AlphaActual / (AlphaActual + distActual[c, r])) * (voroDistActual[c, r] / (distActual[c, r] + voroDistActual[c, r])) * ((DmaxActual - distActual[c, r]) * (DmaxActual - distActual[c, r]) / (DmaxActual * DmaxActual));
        }

        private void rebuildVoronoi()
        {
            while (!voroQ.IsEmpty)
            {
                GridCell s = voroQ.DeleteMin().Position;

                if (patternMatch(s))
                    continue;

                bool stop = false;
                GridCell obstS = obst[s.C, s.R];
                LinkedList<GridCell> neighbors = grid.Get8Neighbors(s);
                foreach (GridCell n in neighbors)
                {
                    GridCell obstN = obst[n.C, n.R];
                    if (chkVoroConditions(s, n, obstS, obstN) && comp[obstS.C, obstS.R] != comp[obstN.C, obstN.R])
                    {
                        stop = true;
                        break;
                    }
                }

                if (stop) continue;

                voro[s.C, s.R] = false;
                foreach (GridCell n in neighbors)
                    if (voro[n.C, n.R])
                        voroQ.Add(new GridCellValue(n, dist[n.C, n.R]));
            }
        }

        private bool patternMatch(GridCell s)
        {
            if (s.C - 1 < 0 || s.R - 1 < 0 || s.C + 1 >= grid.NumColumns || s.R + 1 >= grid.NumRows)
                return false;

            if (voro[s.C - 1, s.R - 1] && !voro[s.C - 1, s.R] && !voro[s.C, s.R - 1]) return true;
            if (voro[s.C - 1, s.R + 1] && !voro[s.C - 1, s.R] && !voro[s.C, s.R + 1]) return true;
            if (voro[s.C + 1, s.R - 1] && !voro[s.C + 1, s.R] && !voro[s.C, s.R - 1]) return true;
            if (voro[s.C + 1, s.R + 1] && !voro[s.C + 1, s.R] && !voro[s.C, s.R + 1]) return true;

            if (voro[s.C - 1, s.R] && voro[s.C + 1, s.R] && !voro[s.C, s.R - 1] && !voro[s.C, s.R + 1]) return true;
            if (!voro[s.C - 1, s.R] && !voro[s.C + 1, s.R] && voro[s.C, s.R - 1] && voro[s.C, s.R + 1]) return true;

            if (voro[s.C - 1, s.R] && voro[s.C + 1, s.R] && voro[s.C, s.R - 1] && voro[s.C, s.R + 1]) return true;

            return false;
        }

        private bool chkVoroConditions(GridCell s, GridCell n, GridCell obstS, GridCell obstN)
        {
            return (dist[s.C, s.R] > 1 || dist[n.C, n.R] > 1) && obstN != GridCell.Unknown && obstN != obstS && !isAdjacent(obstS, obstN);
        }

        private void chkVoro(GridCell s, GridCell n)
        {
            GridCell obstS = obst[s.C, s.R];
            GridCell obstN = obst[n.C, n.R];

            if (comp[obstS.C, obstS.R] == comp[obstN.C, obstN.R]) return;

            if ((dist[s.C, s.R] > 1 || dist[n.C, n.R] > 1) && obst[n.C, n.R] != GridCell.Unknown)
            {
                if (Math.Abs(obstS.C - obstN.C) > 1 || Math.Abs(obstS.R - obstN.R) > 1)
                {
                    int sObstN = distanceSquared(s, obstN);
                    int nObstS = distanceSquared(n, obstS);
                    int sStability = sObstN - dist[s.C, s.R];
                    int nStability = nObstS - dist[n.C, n.R];

                    if (sStability < 0 || nStability < 0)
                        return;

                    if (sStability <= nStability)
                    {
                        voro[s.C, s.R] = true;
                        setVoro(s);
                    }
                    if (nStability <= sStability)
                    {
                        voro[n.C, n.R] = true;
                        setVoro(n);
                    }
                }
            }
        }

        private bool isAdjacent(GridCell s, GridCell n)
        {
            if (s == GridCell.Unknown || n == GridCell.Unknown)
                return false;

            if (s == n)
                return false;

            int dc = s.C - n.C;
            int dr = s.R - n.R;

            return (dc == 1 || dc == 0 || dc == -1) && (dr == 1 || dr == 0 || dr == -1);
        }

        private bool areDiagonal(GridCell s, GridCell n)
        {
            return s.C != n.C && s.R != n.R;
        }

        private bool isOcc(GridCell s)
        {
            return s != GridCell.Unknown && obst[s.C, s.R] == s;
        }

        private bool voroIsOcc(GridCell s)
        {
            return s != GridCell.Unknown && nearestVoro[s.C, s.R] == s;
        }

        private int distanceSquared(GridCell a, GridCell b)
        {
            int dc = a.C - b.C;
            int dr = a.R - b.R;
            return dc * dc + dr * dr;
        }
    }
}
