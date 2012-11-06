using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace AutonomousCar.PathFinding
{
    public enum Steer
    {
        Left,
        Straight,
        Right
    }

    public enum Gear
    {
        Forward,
        Backward
    }

    /// <summary>
    /// The VehicleModel class is used to expand a vehicle pose to another pose based on speed and steering.
    /// </summary>
    public static class VehicleModel
    {
        public const int NumSteers = 3;
        public const int NumGears = 2;

        public const float MaxTurnAngle = 0.43633f; // in radians, 25 degrees
        public const float AxleDistance = 2.93f; // in meters

        // Precomputed for 5 MPH and 25 degree turns
        public const float TurnRadius = 8.0501f; //6.2834f; // in meters
        public const float SlowVelocity = 2.2352f; // in m/s, 5 MPH

        private static Random r = new Random();

        public static Pose NextPose(Pose current, float steerAngle, float velocity, float dt)
        {
            float length = velocity * dt;

            float turnRadius = AxleDistance / (float)Math.Tan(steerAngle);
            float phi = length / turnRadius;
            float phiover2 = phi / 2;
            float sinPhi = (float)Math.Sin(phiover2);
            float L = 2 * sinPhi * turnRadius;
            float x = L * (float)Math.Cos(phiover2);
            float y = L * sinPhi;

            Vector2 pos = new Vector2(x, y);
            pos = Vector2.Transform(pos, Matrix.CreateRotationZ(current.Orientation));

            return new Pose(current.Position + pos, current.Orientation + phi);
        }

        public static Pose NextPose(Pose current, Steer steer, Gear gear, float dt, out float length)
        {
            return NextPose(current, steer, gear, SlowVelocity, dt, TurnRadius, out length);
        }

        public static Pose NextPose(Pose current, Steer steer, Gear gear, float speed, float dt, float turnRadius, out float length)
        {
            length = speed * dt;
            float x, y, phi;
            if (steer == Steer.Straight)
            {
                x = length;
                y = 0;
                phi = 0;
            }
            else
            {
                phi = length / turnRadius;
                float phiover2 = phi / 2;
                float sinPhi = (float)Math.Sin(phiover2);
                float L = 2 * sinPhi * turnRadius;
                x = L * (float)Math.Cos(phiover2);
                y = L * sinPhi;
            }

            if (steer == Steer.Right)
            {
                y = -y;
                phi = -phi;
            }

            if (gear == Gear.Backward)
            {
                x = -x;
                phi = -phi;
            }

            Vector2 pos = new Vector2(x, y);
            pos = Vector2.Transform(pos, Matrix.CreateRotationZ(current.Orientation));

            return new Pose(current.Position + pos, current.Orientation + phi);
        }

        public static Pose RandomContinuousAction(Pose current, float dt)
        {
            return NextPose(current, ((float)r.NextDouble() * 2 - 1) * MaxTurnAngle, SlowVelocity, dt);
        }

        public static Pose RandomAction(Pose current, float dt)
        {
            float l;
            return NextPose(current, (Steer)r.Next(3), Gear.Forward, dt, out l);
        }
    }
}