using System;
using C5;
using Microsoft.Xna.Framework;
using AutonomousCar.PathFinding;
using FarseerPhysics.DebugViews;

namespace AutonomousCar.PathFollowing
{
    /// <summary>
    /// The CarControls struct stores a set of gas, brake, and steering controls.
    /// </summary>
    public struct CarControls
    {
        public float Gas, Brake, Steer;
        public CarControls(float gas, float brake, float steer)
        {
            Gas = gas;
            Brake = brake;
            Steer = steer;
        }
    }

    /// <summary>
    /// The CarController is an abstract base class for car controllers.
    /// </summary>
    public abstract class CarController
    {
        public float MaxSpeed { get; set; } // meters per second
        public Vector2 ClosestPoint { get; protected set; }
        public float CrossTrackError { get; protected set; }
        public ArrayList<Pose> Path { get; set; }

        protected Pose goal;

        public CarController(ArrayList<Pose> path, Pose goal)
        {
            Path = path;
            MaxSpeed = 0.5f;
            this.goal = goal;
        }

        public abstract CarControls Update(Pose currentPose, float wheelAngle, float speed, GameTime gameTime);
        public virtual void Draw(DebugViewXNA draw) { }
    }
}
