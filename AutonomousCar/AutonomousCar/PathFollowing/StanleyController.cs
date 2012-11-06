using System;
using AutonomousCar.Entities;
using C5;
using Microsoft.Xna.Framework;
using AutonomousCar.PathFinding;
using FarseerPhysics.DebugViews;

namespace AutonomousCar.PathFollowing
{
    /// <summary>
    /// The StanleyController class is an older version of the currently used StanleyFSMController. It works similar to the newer version; however, it cannot drive backwards.
    /// This class was replaced with the newer StanleyFSMController when the car controller finite state machine was designed. This class is no longer used.
    /// </summary>
    public class StanleyController : CarController
    {
        public const float Deceleration = 0.5f; // m/s^2
        public const float LateralAcceleration = 0.5f; // m/s^2
        public const int CTEMovingAverageNum = 20;

        public ArrayList<Pose> FrontPath { get; set; }
        public ArrayList<Pose> ReverseFrontPath { get; set; }
        public int NextWaypointIndex { get; set; }
        public int PrevWaypointIndex { get; set; }
        public Vector2 PrevWaypoint { get; set; }
        public Vector2 NextWaypoint { get; set; }
        public Vector2 Left { get; set; }
        public Vector2 Right { get; set; }
        public Vector2 FakeFrontAxle { get; set; }
        public float[] Velocities { get; set; }

        private float vpasterror = 0f;
        private float[] cteAverage;
        private int cteAverageIndex;
        private float prevPhiError = 0f;
        private int lastCusp = 0;

        public StanleyController(ArrayList<Pose> path, Pose goal)
            : base(path, goal)
        {
            MaxSpeed = 4.4704f;

            cteAverageIndex = 0;
            cteAverage = new float[CTEMovingAverageNum];
            for (int i = 0; i < CTEMovingAverageNum; i++)
                cteAverage[i] = 0f;

            FrontPath = new ArrayList<Pose>();
            ReverseFrontPath = new ArrayList<Pose>();

            // Build the front axle path
            int count = Path.Count;
            for (int i = 0; i < count - 1; i++)
            {
                Pose currPose = Path[i];
                Vector2 displacement = Path[i + 1].Position - currPose.Position;
                float pathOrientation = (float)Math.Atan2(displacement.Y, displacement.X);
                FrontPath.Add(new Pose(Car.GetFrontAxlePosition(new Pose(currPose.Position, pathOrientation)), pathOrientation, currPose.Gear));
            }
            float lastOrientation = Path.Last.Gear == Gear.Forward ? Path.Last.Orientation : MathHelper.WrapAngle(Path.Last.Orientation + MathHelper.Pi);
            FrontPath.Add(new Pose(Car.GetFrontAxlePosition(new Pose(Path.Last.Position, lastOrientation)), lastOrientation, Path.Last.Gear));

            // Build front axle path for going in reverse
            for (int i = 0; i < count; i++)
            {
                Pose frontPose = FrontPath[i];
                Vector2 fakeFrontPosition = 2 * Path[i].Position - frontPose.Position;
                ReverseFrontPath.Add(new Pose(fakeFrontPosition, frontPose.Orientation, frontPose.Gear));
            }
                
            NextWaypointIndex = 1;
            PrevWaypointIndex = 0;
            NextWaypoint = path[1].Position;
            PrevWaypoint = path[0].Position;
            ClosestPoint = PrevWaypoint;

            // Determine a recommended velocity for each point
            Velocities = new float[count];
            Velocities[count - 1] = 0f;

            for (int i = count - 2; i >= 0; i--)
            {
                // The velocity should be 0 if the car is about to switch gears
                if (Path[i].Gear != Path[i + 1].Gear)
                {
                    Velocities[i] = 0f;
                    continue;
                }

                float decelConstraint = (float)Math.Sqrt(Velocities[i + 1] * Velocities[i + 1] + 2 * Deceleration * (Path[i + 1].Position - Path[i].Position).Length());
                float curvatureConstraint = float.PositiveInfinity;
                if (i > 0)
                {
                    Vector2 curr = Path[i].Position - Path[i - 1].Position;
                    Vector2 next = Path[i + 1].Position - Path[i].Position;
                    float radius = curr.Length() / (float)Math.Abs(MathHelper.WrapAngle((float)(Math.Atan2(next.Y, next.X) - Math.Atan2(curr.Y, curr.X))));
                    curvatureConstraint = (float)Math.Sqrt(radius * LateralAcceleration);
                }
                Velocities[i] = MathHelper.Min(decelConstraint, MathHelper.Min(curvatureConstraint, Path[i].Gear == Gear.Forward ? MaxSpeed : 0.25f * MaxSpeed));
            }
        }

        public override CarControls Update(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            int prevIndex, nextIndex;
            localize(currentPose.Position, speed, out prevIndex, out nextIndex);

            bool goingBackwards = FrontPath[nextIndex].Gear == Gear.Backward;

            float cte;
            float howFar = howFarAlong(currentPose.Position, Path[prevIndex].Position, Path[nextIndex].Position, out cte);
            cteAverage[cteAverageIndex++] = cte;
            if (cteAverageIndex >= CTEMovingAverageNum) cteAverageIndex = 0;
            float totalCte = 0f;
            for (int i = 0; i < CTEMovingAverageNum; i++)
                totalCte += cteAverage[i];
            CrossTrackError = totalCte / CTEMovingAverageNum;

            NextWaypointIndex = nextIndex;
            PrevWaypointIndex = prevIndex;
            NextWaypoint = Path[nextIndex].Position;
            PrevWaypoint = Path[prevIndex].Position;
            Vector2 next = FrontPath[nextIndex].Position;
            Vector2 prev = FrontPath[prevIndex].Position;

            FakeFrontAxle = Car.GetFakeFrontAxlePosition(currentPose);
            Vector2 frontAxle = goingBackwards ? FakeFrontAxle : Car.GetFrontAxlePosition(currentPose);

            ClosestPoint = findClosestPoint(frontAxle, prev, next);
            Vector2 heading = next - prev;
            float desiredHeading = (float)Math.Atan2(heading.Y, heading.X);

            float nextHeading;
            if (nextIndex >= FrontPath.Count - 1)
            {
                nextHeading = goal.Orientation;
                if (goingBackwards)
                    nextHeading = MathHelper.WrapAngle(goal.Orientation + MathHelper.Pi);
            }
            else if (Path[nextIndex].Gear != Path[nextIndex + 1].Gear)
            {
                nextHeading = MathHelper.WrapAngle(goal.Orientation + MathHelper.Pi);
            }
            else
            {
                Vector2 nextHeadingVec = FrontPath[nextIndex + 1].Position - next;
                nextHeading = (float)Math.Atan2(nextHeadingVec.Y, nextHeadingVec.X);
            }

            // lerp based on how far you are along a segment
            // use WrapAngle to avoid pi - (-pi) issues
            desiredHeading += MathHelper.WrapAngle(nextHeading - desiredHeading) * howFar;

            Vector2 norm = heading;
            norm.Normalize();
            Left = Vector2.Transform(norm, Matrix.CreateRotationZ(MathHelper.PiOver2)) * 2 + ClosestPoint;
            Right = Vector2.Transform(norm, Matrix.CreateRotationZ(-MathHelper.PiOver2)) * 2 + ClosestPoint;

            float dir = 1;
            if ((Left - frontAxle).LengthSquared() < (Right - frontAxle).LengthSquared())
                dir = -1;

            float k = 0.1f;
            float dist = (frontAxle - ClosestPoint).Length();
            float dtheta = MathHelper.WrapAngle(currentPose.Orientation - desiredHeading + (goingBackwards ? MathHelper.Pi : 0));
            if (goingBackwards)
            {
                dtheta = -dtheta;
                dir = -dir;
            }
            float phi = -dtheta + (float)Math.Atan(k * dir * dist);
            if (speed == 0f)
                phi = 0;

            float phiError = phi - wheelAngle;
            float dPhiError = (phiError - prevPhiError) / (float)gameTime.TotalGameTime.TotalSeconds;
            float steer = phiError * 8f + dPhiError * 0.5f;

            // Velocity control
            float vp = 0.5f;
            float vi = 0.00005f;
            float gas = 0f;
            float brake = 0f;
            float verror = speed - MathHelper.Lerp(Velocities[prevIndex], Velocities[nextIndex], howFar);
            vpasterror += verror * (float)gameTime.TotalGameTime.TotalSeconds;
            float vtotalerror = vp * verror + vi * vpasterror;
            if (vtotalerror > 0f)
                brake = vtotalerror;
            else if (vtotalerror < 0f)
                gas = -vtotalerror;

            if (FrontPath[nextIndex].Gear == Gear.Backward)
                gas = -gas;

            return new CarControls(gas, brake, steer);
        }

        public override void Draw(DebugViewXNA draw)
        {
            for (int i = 0; i < Path.Count - 1; i++)
            {
                Color color = Color.Lerp(Color.Red, Color.Green, Velocities[i] / MaxSpeed);
                draw.DrawSegment(Path[i].Position, Path[i + 1].Position, color, 0.04f);
            }

            //draw.DrawCircle(FrontPath[NextWaypointIndex].Position, 0.1f, Color.Orange);
            //draw.DrawCircle(FrontPath[PrevWaypointIndex].Position, 0.1f, Color.Orange);
        }

        private void localize(Vector2 backAxle, float speed, out int prevIndex, out int nextIndex)
        {
            // Find closest path point to back axle
            float bestd = float.MaxValue;
            int besti = 0;
            int start = Math.Max(lastCusp, Math.Max(0, NextWaypointIndex - 2));
            int end = Math.Min(Path.Count - 1, NextWaypointIndex + 2);
            for (int i = start; i < end; i++)
            {
                float d = (backAxle - Path[i].Position).Length();
                if (d < bestd)
                {
                    bestd = d;
                    besti = i;
                }
            }

            nextIndex = besti;
            if (besti + 1 >= Path.Count)
            {
                prevIndex = besti - 1;
            }
            else if (besti - 1 < 0)
            {
                prevIndex = nextIndex;
                nextIndex = besti + 1;
            }
            else
            {
                Vector2 prevmaybe = Path[besti - 1].Position;
                Vector2 nextmaybe = Path[besti + 1].Position;

                if ((backAxle - prevmaybe).LengthSquared() < (backAxle - nextmaybe).LengthSquared())
                {
                    prevIndex = besti - 1;
                }
                else
                {
                    prevIndex = nextIndex;
                    nextIndex = besti + 1;
                }
            }

            /*if (nextIndex + 1 < FrontPath.Count && FrontPath[nextIndex].Gear != FrontPath[nextIndex + 1].Gear && speed < 0.01f)
            {
                nextIndex++;
                prevIndex++;
                lastCusp = nextIndex;
            }*/
        }

        private Vector2 findClosestPoint(Vector2 frontAxle, Vector2 prev, Vector2 next)
        {
            float x, y;
            if (next.X == prev.X) // Avoid division by zero
            {
                x = next.X;
                y = frontAxle.Y;
            }
            else
            {
                float m = (next.Y - prev.Y) / (next.X - prev.X);
                float b = next.Y - m * next.X;
                x = (m * frontAxle.Y + frontAxle.X - m * b) / (m * m + 1);
                y = (m * m * frontAxle.Y + m * frontAxle.X + b) / (m * m + 1);
            }

            return new Vector2(x, y);
        }

        private float howFarAlong(Vector2 rearAxle, Vector2 prev, Vector2 next, out float cte)
        {
            Vector2 r = rearAxle - prev;
            Vector2 d = next - prev;
            cte = Math.Abs((r.Y * d.X + r.X * d.Y) / (d.X * d.X + d.Y * d.Y));
            return (r.X * d.X + r.Y * d.Y) / (d.X * d.X + d.Y * d.Y);
        }
    }
}