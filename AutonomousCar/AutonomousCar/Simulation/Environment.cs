using System;
using FarseerPhysics.Collision;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Factories;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Common;
using C5;
using AutonomousCar.Entities;
using Microsoft.Xna.Framework;

namespace AutonomousCar.Simulation
{
    /// <summary>
    /// The Environment class defines the obstacles in an environment as well as parameters for the obstacle grid.
    /// </summary>
    public class Environment
    {
        public LinkedList<Obstacle> Obstacles { get; set; }
        public LinkedList<CompositeObstacle> CompositeObstacles { get; set; }

        public float GridResolution { get; set; }
        public float GridWidth { get; set; }
        public float GridHeight { get; set; }
        public Vector2 GridOrigin { get; set; }

        public Environment()
        {
            Obstacles = new LinkedList<Obstacle>();
            CompositeObstacles = new LinkedList<CompositeObstacle>();

            GridResolution = 0.75f;
            GridWidth = 150f;
            GridHeight = 150f;
            GridOrigin = Vector2.Zero;
        }
    }

    /// <summary>
    /// The RandomEnvironment class describes an environment with some number of randomly generated obstacles.
    /// </summary>
    public class RandomEnvironment : Environment
    {
        public RandomEnvironment(World world, int numBoxes, Vector2 start, Vector2 destination, float envWidth, float envHeight)
        {
            GridWidth = envWidth;
            GridHeight = envHeight;

            generateEnvironment(world, numBoxes, start, destination);
        }

        public RandomEnvironment(World world, int numBoxes, Vector2 start, Vector2 destination)
        {
            generateEnvironment(world, numBoxes, start, destination);
        }

        private void generateEnvironment(World world, int numBoxes, Vector2 start, Vector2 destination)
        {
            Random r = new Random();

            Vector2 pos;
            float size;
            float orientation;
            for (int i = 0; i < numBoxes; i++)
            {
                do
                {
                    size = (float)r.NextDouble() * 3f + 1f;
                    pos = new Vector2((float)r.NextDouble() * GridWidth, (float)r.NextDouble() * GridHeight);
                    orientation = (float)r.NextDouble() * MathHelper.TwoPi;
                } while ((pos - start).LengthSquared() < 100f || (pos - destination).LengthSquared() < 100f || !checkObstacle(size, pos, orientation));
                Obstacles.Add(new BoxObstacle(world, size, size, pos, orientation));
            }
        }

        private bool checkObstacle(float size, Vector2 pos, float orientation)
        {
            PolygonShape shape = new PolygonShape(1f);
            shape.SetAsBox(size, size);
            Transform xform = new Transform();
            xform.Set(pos, orientation);

            foreach (Obstacle obs in Obstacles)
            {
                Body obsBody = obs.Body;
                Transform obstXform;
                obsBody.GetTransform(out obstXform);
                if (obsBody.FixtureList != null && AABB.TestOverlap(shape, 0, obsBody.FixtureList[0].Shape, 0, ref xform, ref obstXform))
                    return false;
            }

            return true;
        }
    }

    /// <summary>
    /// The ParkingEnvironment class describes a basic parking lot type environment.
    /// </summary>
    public class ParkingEnvironment : Environment
    {
        public ParkingEnvironment(World world) : base()
        {
            Random r = new Random();

            for (int i = 0; i < 50; i++)
                if (r.NextDouble() < 0.98)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(-10 + i * 2.5f, 30)));
            for (int i = 0; i < 50; i++)
                if (r.NextDouble() < 0.98)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(-10 + i * 2.5f, 35)));

            for (int i = 0; i < 20; i++)
                if (r.NextDouble() < 0.98)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(20 + i * 2.5f, 60)));
            for (int i = 0; i < 20; i++)
                if (r.NextDouble() < 0.98)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(20 + i * 2.5f, 55)));

            for (int i = 0; i < 40; i++)
                if (r.NextDouble() < 0.98)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(80 + i * 2.5f, 60)));
            for (int i = 0; i < 40; i++)
                if (r.NextDouble() < 0.98)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(80 + i * 2.5f, 55)));

            for (int i = 0; i < 40; i++)
                if (r.NextDouble() < 0.95)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(-10 + i * 2.5f, 90)));
            for (int i = 0; i < 40; i++)
                if (r.NextDouble() < 0.95)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(-10 + i * 2.5f, 85)));

            for (int i = 0; i < 40; i++)
                if (r.NextDouble() < 0.95)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(90f + i * 2.5f, 90)));
            for (int i = 0; i < 40; i++)
                if (r.NextDouble() < 0.95)
                    Obstacles.Add(new BoxObstacle(world, 2 * Car.HALF_CAR_WIDTH, 2 * Car.HALF_CAR_LENGTH, new Vector2(92.5f + i * 2.5f, 85)));
        }
    }
}
