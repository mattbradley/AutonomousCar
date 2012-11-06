using System;
using C5;
using AutonomousCar.PathFinding;
using Microsoft.Xna.Framework;

namespace AutonomousCar.PathFollowing
{
    /// <summary>
    /// The PIDController class is an unused type of car controller. In it's current state, it is very inaccurate; therefore, it was replaced by the StanleyFSMController.
    /// </summary>
    public class PIDController : CarController
    {
        public float PGain { get; set; }
        public float DGain { get; set; }
        public bool CheckpointReached { get; private set; }
        public Vector2 left { get; set; }
        public Vector2 right { get; set; }

        private float previouscte = 0f;
        private bool stop = false;

        public PIDController(ArrayList<Pose> path, Pose goal)
            : base(path, goal)
        {
            CheckpointReached = false;
            MaxSpeed = 2.2352f;
            PGain = 0.5f;
            DGain = 1f;
        }

        public override CarControls Update(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            if ((Path.Last.Position - currentPose.Position).LengthSquared() < 2f || stop)
            {
                stop = true;
                if (speed == 0f) CheckpointReached = true;
                return new CarControls(0f, 0.1f, 0f);
            }

            // Find closest path point to rear axle
            Vector2 rearAxle = currentPose.Position;
            float frontbestd = float.MaxValue;
            int frontbesti = 0;
            for (int i = 0; i < Path.Count - 1; i++)
            {
                float d = (rearAxle - Path[i].Position).LengthSquared();
                if (d < frontbestd)
                {
                    frontbestd = d;
                    frontbesti = i;
                }
            }

            Vector2 next = Path[frontbesti].Position;
            Vector2 prev;
            if (frontbesti + 1 >= Path.Count)
                prev = Path[frontbesti - 1].Position;
            else if (frontbesti - 1 < 0)
            {
                prev = next;
                next = Path[frontbesti + 1].Position;
            }
            else
            {
                Vector2 prevmaybe = Path[frontbesti - 1].Position;
                Vector2 nextmaybe = Path[frontbesti + 1].Position;

                if ((rearAxle - prevmaybe).LengthSquared() < (rearAxle - nextmaybe).LengthSquared())
                    prev = prevmaybe;
                else
                {
                    prev = next;
                    next = nextmaybe;
                }
            }

            /*float Rx = rearAxle.X - prev.X;
            float Ry = rearAxle.Y - prev.Y;
            float dx = next.X - prev.X;
            float dy = next.Y - prev.Y;
            float cte = (Ry * dx + Rx * dy) / (dx * dx + dy * dy);*/

            // Find the closest point to the front axle on the line defined by the points next and prev
            float x, y;
            if (next.X == prev.X) // Avoid division by zero
            {
                x = next.X;
                y = rearAxle.Y;
            }
            else
            {
                float m = (next.Y - prev.Y) / (next.X - prev.X);
                float b = next.Y - m * next.X;
                x = (m * rearAxle.Y + rearAxle.X - m * b) / (m * m + 1);
                y = (m * m * rearAxle.Y + m * rearAxle.X + b) / (m * m + 1);
            }

            ClosestPoint = new Vector2(x, y);
            CrossTrackError = (rearAxle - ClosestPoint).Length();

            Vector2 norm = next - prev;
            norm.Normalize();
            left = Vector2.Transform(norm, Matrix.CreateRotationZ(MathHelper.PiOver2)) * 2 + ClosestPoint;
            right = Vector2.Transform(norm, Matrix.CreateRotationZ(-MathHelper.PiOver2)) * 2 + ClosestPoint;

            float dir = 1;
            if ((left - rearAxle).LengthSquared() < (right - rearAxle).LengthSquared())
                dir = -1;

            float alpha = PGain * CrossTrackError + DGain * (CrossTrackError - previouscte) / (float)gameTime.ElapsedGameTime.TotalSeconds;
            previouscte = CrossTrackError;

            float gas = 0;
            if (speed < MaxSpeed)
                gas = 0.5f;

            float steer = alpha * dir - wheelAngle;

            return new CarControls(gas, 0f, steer);
        }
    }
}
