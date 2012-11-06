using System;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using FarseerPhysics.Collision;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Factories;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Common;
using AutonomousCar.Entities;
using C5;

namespace AutonomousCar.PathFinding
{
    public struct GridCell
    {
        public int C;
        public int R;
        public static GridCell Unknown = new GridCell(-1, -1);

        public GridCell(int c, int r)
        {
            C = c;
            R = r;
        }

        public static bool operator ==(GridCell c1, GridCell c2)
        {
            return c1.C == c2.C && c1.R == c2.R;
        }

        public static bool operator !=(GridCell c1, GridCell c2)
        {
            return c1.C != c2.C || c1.R != c2.R;
        }

        public override bool Equals(object obj)
        {
            throw new NotImplementedException();
        }

        public override int GetHashCode()
        {
            throw new NotImplementedException();
        }
    }

    public struct GridCellValue : IComparable<GridCellValue>
    {
        public GridCell Position;
        public IComparable Value;

        public GridCellValue(GridCell cell, IComparable value)
        {
            Position = cell;
            Value = value;
        }

        public int CompareTo(GridCellValue other)
        {
            return Value.CompareTo(other.Value);
        }
    }

    /// <summary>
    /// The ObstacleGrid class indexes all obstacles in the environment into a rasterized grid. It can be used to detect the safety of
    /// a vehicle pose. It also provides an interface to the generalized Voronoi diagram.
    /// </summary>
    public class ObstacleGrid
    {
        public World World { get; set; }
        public float Resolution { get; private set; }
        public float HalfResolution { get; private set; }
        public float DiagonalResolution { get; private set; }
        public int NumColumns { get; set; }
        public int NumRows { get; set; }
        public Vector2 Origin { get; set; }
        public GVDLau GVD { get; private set; }
        public HashDictionary<int, LinkedList<GridCell>> Obstacles { get; private set; }
        public int[,] OccupiedCells { get; set; }
        public LinkedList<Body>[,] ObstaclesByCell { get; set; }
        public LinkedList<Body> BodyList { get; set; }

        private float width;
        private float height;
        private float lowerX;
        private float lowerY;
        private float upperX;
        private float upperY;
        private Texture2D distanceFieldBitmap;
        private Texture2D voronoiFieldBitmap;
        private int nextObstacleId;

        public ObstacleGrid(World world, float resolution, float width, float height, Vector2 origin)
            : this(world, resolution, (int)Math.Ceiling(width / resolution), (int)Math.Ceiling(height / resolution), origin) { }

        public ObstacleGrid(World world, AutonomousCar.Simulation.Environment e)
            : this(world, e.GridResolution, e.GridWidth, e.GridHeight, e.GridOrigin)
        {
            LoadEnvironment(e);
        }

        public ObstacleGrid(World world, float resolution, int numColumns, int numRows, Vector2 origin)
        {
            World = world;
            Resolution = resolution;
            HalfResolution = resolution / 2;
            DiagonalResolution = resolution * (float)Math.Sqrt(2.0);
            NumColumns = numColumns;
            NumRows = numRows;
            Origin = origin;

            width = resolution * numColumns;
            height = resolution * numRows;

            lowerX = origin.X;
            lowerY = origin.Y;
            upperX = origin.X + width;
            upperY = origin.Y + height;

            GVD = new GVDLau(this);
            Obstacles = new HashDictionary<int, LinkedList<GridCell>>();
            nextObstacleId = 0;
            OccupiedCells = new int[numColumns, numRows];
            ObstaclesByCell = new LinkedList<Body>[numColumns, numRows];
            BodyList = new LinkedList<Body>();

            for (int c = 0; c < numColumns; c++)
                for (int r = 0; r < numRows; r++)
                    if (c == 0 || c == numColumns - 1 || r == 0 || r == numRows - 1)
                        OccupiedCells[c, r] = 1;
                    else
                        OccupiedCells[c, r] = 0;
        }

        public int AddObstacle(Obstacle obstacle)
        {
            Obstacles[nextObstacleId] = new LinkedList<GridCell>();
            addBodyToGrid(obstacle.Body, nextObstacleId);
            return nextObstacleId++;
        }

        public int AddCompositeObstacle(CompositeObstacle obstacle)
        {
            Obstacles[nextObstacleId] = new LinkedList<GridCell>();
            foreach (Body body in obstacle.Bodies)
                addBodyToGrid(body, nextObstacleId);
            return nextObstacleId++;
        }

        public void LoadEnvironment(AutonomousCar.Simulation.Environment environment)
        {
            foreach (Obstacle o in environment.Obstacles)
                AddObstacle(o);

            foreach (CompositeObstacle o in environment.CompositeObstacles)
                AddCompositeObstacle(o);
        }

        private void addBodyToGrid(Body body, int obstacleId)
        {
            BodyList.Add(body);
            Vertices vertices = ((PolygonShape)body.FixtureList[0].Shape).Vertices;
            for (int i = 0; i < vertices.Count; i++)
            {
                int to = i == vertices.Count - 1 ? 0 : i + 1;
                foreach (GridCell cell in rasterizeLine(body.GetWorldPoint(vertices[i]), body.GetWorldPoint(vertices[to])))
                {
                    Obstacles[obstacleId].Add(cell);
                    GVD.SetObstacle(cell, obstacleId);
                    OccupiedCells[cell.C, cell.R]++;

                    LinkedList<Body> obs = ObstaclesByCell[cell.C, cell.R];
                    if (obs == null)
                        obs = ObstaclesByCell[cell.C, cell.R] = new LinkedList<Body>();

                    obs.Add(body);
                }
            }
        }

        public void RemoveObstacle(int obstacleId)
        {
            LinkedList<GridCell> cells;
            if (!Obstacles.Find(ref obstacleId, out cells))
                return;

            foreach (GridCell cell in cells)
            {
                GVD.UnsetObstacle(cell);
                OccupiedCells[cell.C, cell.R]--;
            }
        }

        public void UpdateVoronoiFieldParameters(float alpha, float dmax)
        {
            GVD.UpdatePathCostParameters(alpha, dmax);
            voronoiFieldBitmap = null;
        }

        public LinkedList<GridCell> Get4Neighbors(GridCell cell)
        {
            if (cell == GridCell.Unknown)
                return new LinkedList<GridCell>();

            LinkedList<GridCell> neighbors = new LinkedList<GridCell>();

            int c = cell.C;
            int r = cell.R;

            if (c - 1 >= 0) neighbors.Add(new GridCell(c - 1, r));
            if (c + 1 < NumColumns) neighbors.Add(new GridCell(c + 1, r));
            if (r - 1 >= 0) neighbors.Add(new GridCell(c, r - 1));
            if (r + 1 < NumRows) neighbors.Add(new GridCell(c, r + 1));

            return neighbors;
        }

        public LinkedList<GridCell> Get8Neighbors(GridCell cell)
        {
            if (cell == GridCell.Unknown)
                return new LinkedList<GridCell>();

            int c = cell.C;
            int r = cell.R;
            bool cm = c - 1 >= 0;
            bool cp = c + 1 < NumColumns;
            bool rm = r - 1 >= 0;
            bool rp = r + 1 < NumRows;

            LinkedList<GridCell> neighbors = new LinkedList<GridCell>();

            if (cm)
            {
                neighbors.Add(new GridCell(c - 1, r));
                if (rm) neighbors.Add(new GridCell(c - 1, r - 1));
                if (rp) neighbors.Add(new GridCell(c - 1, r + 1));
            }

            if (cp)
            {
                neighbors.Add(new GridCell(c + 1, r));
                if (rm) neighbors.Add(new GridCell(c + 1, r - 1));
                if (rp) neighbors.Add(new GridCell(c + 1, r + 1));
            }

            if (rm) neighbors.Add(new GridCell(c, r - 1));
            if (rp) neighbors.Add(new GridCell(c, r + 1));

            return neighbors;
        }

        public Vector2 GetNearestObstacleCellLocation(Vector2 pos)
        {
            return CellPositionToPoint(GVD.GetNearestObstacle(PointToCellPosition(pos))) + new Vector2(HalfResolution);
        }


        public bool IsSafe(Pose pose) { return IsSafe(pose, 1f); }
        public bool IsSafe(Pose pose, float safetyFactor)
        {
            Vector2 pos = Car.GetCenterPosition(pose);
            GridCell cellPos = PointToCellPosition(pos);
            if (cellPos == GridCell.Unknown) return false;

            float obstDist = GVD.GetObstacleDistance(cellPos);

            //if (obstDist > Car.SAFE_RADIUS * safetyFactor + DiagonalResolution)
                //return true;

            GridCell nearestObstCell = GVD.GetNearestObstacle(cellPos);
            LinkedList<Body> obstacles = ObstaclesByCell[nearestObstCell.C, nearestObstCell.R];

            PolygonShape carShape = new PolygonShape(1f);
            Transform carXform = new Transform();
            carShape.SetAsBox(Car.HALF_CAR_LENGTH * safetyFactor, Car.HALF_CAR_WIDTH * safetyFactor);
            carXform.Set(pos, pose.Orientation);

            foreach (Body obs in obstacles)
            {
                Transform obstXform;
                obs.GetTransform(out obstXform);
                if (obs.FixtureList != null && AABB.TestOverlap(carShape, 0, obs.FixtureList[0].Shape, 0, ref carXform, ref obstXform))
                    return false;
            }

            return true;
        }

        public void BuildGVD()
        {
            GVD.Update();
            distanceFieldBitmap = null;
            voronoiFieldBitmap = null;
        }

        public GridCell PointToCellPosition(Vector2 point)
        {
            return PointToCellPosition(point, false);
        }

        public GridCell PointToCellPosition(Vector2 point, bool notUnknown)
        {
            point -= Origin;
            int c = (int)Math.Floor(point.X / Resolution);
            int r = (int)Math.Floor(point.Y / Resolution);

            if (notUnknown)
                return new GridCell(c, r);

            return c >= 0 && c < NumColumns && r >= 0 && r < NumRows ? new GridCell(c, r) : GridCell.Unknown;
        }

        public Vector2 CellPositionToPoint(GridCell cell)
        {
            return Origin + new Vector2(cell.C, cell.R) * Resolution;
        }

        public bool IsPointInGrid(Vector2 point)
        {
            return point.X >= lowerX && point.Y >= lowerY && point.X < upperX && point.Y < upperY;
        }

        public void Draw(SpriteBatch spriteBatch, bool drawVoronoi)
        {
            if (distanceFieldBitmap == null)
                buildDistanceFieldBitmap(spriteBatch.GraphicsDevice);

            if (voronoiFieldBitmap == null)
                buildVoronoiFieldBitmap(spriteBatch.GraphicsDevice);

            if (drawVoronoi)
                spriteBatch.Draw(voronoiFieldBitmap, Origin, null, Color.White, 0, Vector2.Zero, Resolution, SpriteEffects.None, 0f);
            else
                spriteBatch.Draw(distanceFieldBitmap, Origin, null, Color.White, 0, Vector2.Zero, Resolution, SpriteEffects.None, 0f);
        }

        private void buildDistanceFieldBitmap(GraphicsDevice device)
        {
            distanceFieldBitmap = new Texture2D(device, NumColumns, NumRows);
            Color[] distance = new Color[NumColumns * NumRows];

            for (int c = 0; c < NumColumns; c++)
                for (int r = 0; r < NumRows; r++)
                {
                    float val = GVD.GetObstacleDistance(c, r);
                    if (val == 0f)
                        distance[c + r * NumColumns] = Color.Red;
                    //else if (GVD.GetVoronoiDistance(new GridCell(c, r)) == 0f)
                        //distance[c + r * NumColumns] = Color.White;
                    else
                    {
                        val /= 32;
                        //val = 255;
                        distance[c + r * NumColumns] = new Color(val, val, val);
                    }
                }

            distanceFieldBitmap.SetData<Color>(distance);
        }

        private void buildVoronoiFieldBitmap(GraphicsDevice device)
        {
            voronoiFieldBitmap = new Texture2D(device, NumColumns, NumRows);

            Color[] voro = new Color[NumColumns * NumRows];

            for (int c = 0; c < NumColumns; c++)
                for (int r = 0; r < NumRows; r++)
                {
                    float val = GVD.GetPathCost(c, r);
                    voro[c + r * NumColumns] = new Color(Vector3.One - new Color(val, val, val).ToVector3());
                }

            voronoiFieldBitmap.SetData<Color>(voro);
        }

        private LinkedList<GridCell> rasterizeLine(Vector2 from, Vector2 to)
        {
            // Find cells of the endpoints
            GridCell fromCell = PointToCellPosition(from, true);
            GridCell toCell = PointToCellPosition(to, true);

            int x0 = fromCell.C;
            int y0 = fromCell.R;
            int x1 = toCell.C;
            int y1 = toCell.R;
            int temp;

            LinkedList<GridCell> line = new LinkedList<GridCell>();

            // Bresenham's Line Algorithm
            bool steep = Math.Abs(y1 - y0) > Math.Abs(x1 - x0);
            if (steep)
            {
                // Swap x and y coordinates
                temp = x0;
                x0 = y0;
                y0 = temp;

                temp = x1;
                x1 = y1;
                y1 = temp;
            }

            if (x0 > x1)
            {
                // Swap endpoints
                temp = x0;
                x0 = x1;
                x1 = temp;

                temp = y0;
                y0 = y1;
                y1 = temp;
            }

            int dx = x1 - x0;
            int dy = Math.Abs(y1 - y0);
            int err = dx / 2;
            int ystep = y0 < y1 ? 1 : -1;
            int y = y0;

            for (int x = x0; x <= x1; x++)
            {
                GridCell c;
                if (steep)
                    c = new GridCell(y, x);
                else
                    c = new GridCell(x, y);

                if (c.C >= 0 && c.C < NumColumns && c.R >= 0 && c.R < NumRows)
                    line.Add(c);

                err = err - dy;
                if (err < 0)
                {
                    y += ystep;
                    err += dx;
                }
            }

            return line;
        }
    }
}