using System;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using FarseerPhysics.Common;
using FarseerPhysics.Common.Decomposition;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Factories;
using C5;

namespace AutonomousCar.Entities
{
    /// <summary>
    /// The Obstacle abstract base class.
    /// </summary>
    public abstract class Obstacle
    {
        public Body Body { get; set; }
    }

    /// <summary>
    /// The CompositeObstacle class is a collection of Obstacle objects that can be considered part of one obstacle.
    /// </summary>
    public class CompositeObstacle
    {
        public LinkedList<Body> Bodies { get; set; }

        public CompositeObstacle()
        {
            Bodies = new LinkedList<Body>();
        }

        public CompositeObstacle(C5.ICollection<Obstacle> obstacles)
        {
            Bodies = new LinkedList<Body>();

            foreach (Obstacle o in obstacles)
                Bodies.Add(o.Body);
        }
    }

    /// <summary>
    /// The BoxObstacle class extends Obstacle to provide a rectangular physics obstacle.
    /// </summary>
    public class BoxObstacle : Obstacle
    {
        public BoxObstacle(World world, float width, float height, Vector2 position) : this(world, width, height, position, 0f) { }
        public BoxObstacle(World world, float width, float height, Vector2 position, float rotation)
        {
            Body = BodyFactory.CreateRectangle(world, width, height, 1f, position);
            Body.BodyType = BodyType.Static;
            Body.Rotation = rotation;
        }
    }

    /// <summary>
    /// The CarObstacle class extends Obstacle to provide a car obstacle with a constant size.
    /// </summary>
    public class CarObstacle : Obstacle
    {

        public CarObstacle(World world, Vector2 position) : this(world, position, 0) { }
        public CarObstacle(World world, Vector2 position, float rotation)
        {
            Body = BodyFactory.CreateRectangle(world, Car.HALF_CAR_LENGTH * 2, Car.HALF_CAR_WIDTH * 2, 1f, position);
            Body.BodyType = BodyType.Static;
            Body.Rotation = rotation;
        }
    }

    /// <summary>
    /// The ParkingSpaceObstacle extends CompositeObstacle to provide a row of car obstacles.
    /// </summary>
    public class ParkingSpaceObstacle : CompositeObstacle
    {
        public ParkingSpaceObstacle(World world, Vector2 position) : this(world, position, 0) { }
        public ParkingSpaceObstacle(World world, Vector2 position, float rotation)
        {
            Vector2[] vertices = new Vector2[8];
            vertices[0] = new Vector2(-3.5f, 1.5f);
            vertices[1] = new Vector2(3.5f, 1.5f);
            vertices[2] = new Vector2(3.5f, -1.5f);
            vertices[3] = new Vector2(-3.5f, -1.5f);
            vertices[4] = new Vector2(-3.5f, -1.45f);
            vertices[5] = new Vector2(3.45f, -1.45f);
            vertices[6] = new Vector2(3.45f, 1.45f);
            vertices[7] = new Vector2(-3.45f, 1.45f);

            for (int i = 0; i < 8; i++)
                vertices[i] *= 10;

            LinkedList<Vertices> vertexSet = new LinkedList<Vertices>();
            vertexSet.AddAll(EarclipDecomposer.ConvexPartition(new Vertices(vertices)));

            foreach (Vertices v in vertexSet)
            {
                Body b = BodyFactory.CreatePolygon(world, v, 1f, position);
                b.BodyType = BodyType.Static;
                b.Rotation = rotation;

                Bodies.Add(b);
            }
        }
    }
}