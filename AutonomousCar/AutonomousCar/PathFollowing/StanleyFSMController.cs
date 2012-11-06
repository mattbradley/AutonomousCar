using System;
using System.Linq;
using AutonomousCar.Entities;
using C5;
using Microsoft.Xna.Framework;
using AutonomousCar.PathFinding;
using FarseerPhysics.DebugViews;

namespace AutonomousCar.PathFollowing
{
    /// <summary>
    /// The ControllerInfo class describes statistics taken by the car controller during driving.
    /// It is mainly used by the Experiment class.
    /// </summary>
    public class ControllerInfo : EventArgs
    {
        public float AverageCTE { get; set; }
        public float AverageSpeed { get; set; }
        public float AverageLateralAcceleration { get; set; }
        public float MaxLateralAcceleration { get; set; }
        public float AverageDLateralAcceleration { get; set; }
        public float GoalPositionError { get; set; }
        public float GoalOrientationError { get; set; }
    }

    /// <summary>
    /// The StanleyFSMController is an updated version of StanleyController with the addition of the finite state machine allowing the vehicle to properly drive in reverse.
    /// This is the current car controller used by the simulation.
    /// </summary>
    public class StanleyFSMController : CarController
    {
        public enum ControllerState
        {
            MissionStart,
            Stopped,
            ForwardDrive,
            ReverseDrive,
            MissionComplete
        }

        public const float Deceleration = 0.25f; // m/s^2
        public const float LateralAcceleration = 0.5f; // m/s^2

        public float MaxReverseSpeed { get; set; }
        public ArrayList<Pose> FrontPath { get; set; }
        public ArrayList<Pose> ReverseFrontPath { get; set; }
        public int NextWaypointIndex { get; set; }
        public int PrevWaypointIndex { get; set; }
        public Vector2 Left { get; set; }
        public Vector2 Right { get; set; }
        public Vector2 FakeFrontAxle { get; set; }
        public float[] Velocities { get; set; }
        public bool InReverse { get; set; }
        public string DebugInfo { get; set; }

        private LinkedList<int> stoppingPoints;
        private float vpasterror = 0f;
        private float totalCte;
        private float totalLateralAccel;
        private float maxLateralAccel;
        private float totalSpeed;
        private float totalDLateralAccel;
        private float lastLateralAccel;
        private int countInfo;
        private float prevPhiError = 0f;
        private int lastCusp = 0;
        private Pose lastPose;

        public ControllerState State { get; set; }
        public ControllerInfo Info
        {
            get
            {
                return new ControllerInfo()
                {
                    AverageCTE = totalCte / countInfo,
                    AverageSpeed = totalSpeed / countInfo,
                    AverageLateralAcceleration = totalLateralAccel / countInfo,
                    MaxLateralAcceleration = maxLateralAccel,
                    AverageDLateralAcceleration = totalDLateralAccel / countInfo,
                    GoalPositionError = (goal.Position - lastPose.Position).Length(),
                    GoalOrientationError = (float)Math.Abs(goal.Orientation - lastPose.Orientation)
                };
            }
        }

        public StanleyFSMController(ArrayList<Pose> path, Pose goal) : base(path, goal)
        {
            MaxSpeed = 100f;// 4.4704f;
            MaxReverseSpeed = MaxSpeed / 4;
            State = ControllerState.MissionStart;
            stoppingPoints = new LinkedList<int>();
            DebugInfo = "";

            lastPose = new Pose();
            totalCte = 0f;
            totalLateralAccel = 0f;
            maxLateralAccel = 0f;
            lastLateralAccel = 0f;
            countInfo = 0;

            if (Path.Count == 0) return;

            MissionStart();
        }

        public CarControls MissionStart()
        {
            FrontPath = new ArrayList<Pose>();
            ReverseFrontPath = new ArrayList<Pose>();

            FrontPath.Add(new Pose(Car.GetFrontAxlePosition(Path.First), Path.First.Orientation, Path.First.Gear));
            ReverseFrontPath.Add(new Pose(Car.GetFakeFrontAxlePosition(Path.First), Path.First.Orientation, Path.First.Gear));
            int count = Path.Count;
            for (int i = 1; i < count - 1; i++)
            {
                Pose currPose = Path[i], prevPose = Path[i - 1], nextPose = Path[i + 1];

                if (currPose.Gear != prevPose.Gear)
                {
                    stoppingPoints.Add(i);
                    FrontPath.Add(new Pose(Car.GetFrontAxlePosition(currPose), currPose.Orientation, currPose.Gear));
                    ReverseFrontPath.Add(new Pose(Car.GetFakeFrontAxlePosition(currPose), currPose.Orientation, currPose.Gear));
                }
                else
                {
                    Vector2 displacement = 0.25f * (nextPose.Position - prevPose.Position) + 0.75f * (nextPose.Position - currPose.Position);
                    float pathOrientation = (float)Math.Atan2(displacement.Y, displacement.X);
                    if (currPose.Gear == Gear.Backward)
                        pathOrientation = MathHelper.WrapAngle(pathOrientation + MathHelper.Pi);

                    FrontPath.Add(new Pose(Car.GetFrontAxlePosition(new Pose(currPose.Position, pathOrientation)), pathOrientation, currPose.Gear));
                    ReverseFrontPath.Add(new Pose(Car.GetFakeFrontAxlePosition(new Pose(currPose.Position, pathOrientation)), pathOrientation, currPose.Gear));
                }
            }

            FrontPath.Add(new Pose(Car.GetFrontAxlePosition(Path.Last), Path.Last.Orientation, Path.Last.Gear));
            ReverseFrontPath.Add(new Pose(Car.GetFakeFrontAxlePosition(Path.Last), Path.Last.Orientation, Path.Last.Gear));
            stoppingPoints.Add(count - 1);

            // Determine a recommended velocity for each point
            Velocities = new float[count];
            Velocities[count - 1] = 0.1f;

            for (int i = count - 2; i >= 0; i--)
            {
                // The velocity should be low if the car is about to switch gears
                if (i > 0 && Path[i - 1].Gear != Path[i].Gear)
                {
                    Velocities[i] = 0.2f;
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
                Velocities[i] = MathHelper.Min(decelConstraint, MathHelper.Min(curvatureConstraint, Path[i].Gear == Gear.Forward ? MaxSpeed : MaxReverseSpeed));
            }

            NextWaypointIndex = 1;
            PrevWaypointIndex = 0;
            ClosestPoint = Path[0].Position;

            State = ControllerState.Stopped;

            return new CarControls();
        }

        public CarControls Stopped(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            float phiError;
            float desiredHeading;
            ControllerState nextState;

            if (NextWaypointIndex == Path.Count - 1 && PrevWaypointIndex == Path.Count - 1)
            {
                desiredHeading = currentPose.Orientation;
                nextState = ControllerState.MissionComplete;
            }
            else
            {
                ArrayList<Pose> thePath = FrontPath[PrevWaypointIndex].Gear == Gear.Forward ? FrontPath : ReverseFrontPath;
                Vector2 next = thePath[NextWaypointIndex].Position;
                Vector2 prev = thePath[PrevWaypointIndex].Position;
                Vector2 heading = next - prev;
                desiredHeading = (float)Math.Atan2(heading.Y, heading.X);
                nextState = thePath[PrevWaypointIndex].Gear == Gear.Forward ? ControllerState.ForwardDrive : ControllerState.ReverseDrive;
            }

            phiError = -MathHelper.WrapAngle(currentPose.Orientation - desiredHeading) - wheelAngle;
            if (Math.Abs(phiError) < 0.001f || Math.Abs(Math.Abs(wheelAngle) - Car.MAX_WHEEL_DEFLECTION) < 0.1f)
            {
                State = nextState;
                Velocities[PrevWaypointIndex] = Velocities[NextWaypointIndex];
            }

            return new CarControls(0f, 1f, phiError * 8f);
        }

        public CarControls ForwardDrive(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            int prevIndex, nextIndex;
            localize(currentPose.Position, speed, out prevIndex, out nextIndex);

            InReverse = FrontPath[prevIndex].Gear == Gear.Backward;
            bool comingToStop = stoppingPoints.Contains(nextIndex);

            float cte;
            float howFar = MathHelper.Clamp(howFarAlong(currentPose.Position, Path[prevIndex].Position, Path[nextIndex].Position, out cte), 0f, 1f);
            DebugInfo = String.Format("How far: {0:0.000}\n", howFar);
            updateInfo(cte, wheelAngle, speed, gameTime);

            ArrayList<Pose> thePath = InReverse ? ReverseFrontPath : FrontPath;

            NextWaypointIndex = nextIndex;
            PrevWaypointIndex = prevIndex;
            Vector2 next = thePath[nextIndex].Position;
            Vector2 prev = thePath[prevIndex].Position;

            FakeFrontAxle = Car.GetFakeFrontAxlePosition(currentPose);
            Vector2 frontAxle = InReverse ? FakeFrontAxle : Car.GetFrontAxlePosition(currentPose);

            ClosestPoint = findClosestPoint(frontAxle, prev, next);
            Vector2 heading = prevIndex > 0 ? next - thePath[prevIndex - 1].Position : next - prev;
            float desiredHeading = (float)Math.Atan2(heading.Y, heading.X);

            float nextHeading;
            if (comingToStop)
            {
                nextHeading = Path[nextIndex].Orientation + (InReverse ? MathHelper.Pi : 0);
            }
            else
            {
                Vector2 nextHeadingVec = thePath[nextIndex + 1].Position - prev;
                nextHeading = (float)Math.Atan2(nextHeadingVec.Y, nextHeadingVec.X);
            }

            // lerp based on how far you are along a segment
            // use WrapAngle to avoid pi - (-pi) issues
            desiredHeading += MathHelper.WrapAngle(nextHeading - desiredHeading) * howFar;
            DebugInfo += String.Format("Orientation: {0:0.000}\n", desiredHeading);

            // Is the path to our left or right?
            Vector2 norm = next - prev;
            norm.Normalize();
            Left = Vector2.Transform(norm, Matrix.CreateRotationZ(MathHelper.PiOver2)) * 2 + ClosestPoint;
            Right = Vector2.Transform(norm, Matrix.CreateRotationZ(-MathHelper.PiOver2)) * 2 + ClosestPoint;

            float dir = 1;
            if ((Left - frontAxle).LengthSquared() < (Right - frontAxle).LengthSquared())
                dir = -1;

            float k = 1.5f;
            float dist = (frontAxle - ClosestPoint).Length();
            float dtheta = MathHelper.WrapAngle(currentPose.Orientation - desiredHeading + (InReverse ? MathHelper.Pi : 0));
            if (InReverse)
            {
                dtheta = -dtheta;
                dir = -dir;
            }

            float phi;
            if (comingToStop)
                phi = (float)Math.Atan(4f * k * dir * dist);
            else
                phi = -dtheta + (float)Math.Atan(k * dir * dist);

            float phiError = phi - wheelAngle;
            float dPhiError = (phiError - prevPhiError) / (float)gameTime.TotalGameTime.TotalSeconds;
            float steer = phiError * 8f + dPhiError * 0.5f;

            // Velocity control
            float vp = 0.5f;
            float vi = 0f;// 0.00005f;
            float gas = 0f;
            float brake = 0f;
            float verror = speed - MathHelper.Lerp(Velocities[prevIndex], Velocities[nextIndex], howFar);
            DebugInfo += String.Format("Speed: {0:0.000}\n", speed);
            DebugInfo += String.Format("Target speed: {0:0.000}\n", MathHelper.Lerp(Velocities[prevIndex], Velocities[nextIndex], howFar));
            DebugInfo += String.Format("From {0:0.000} to {1:0.000}", Velocities[prevIndex], Velocities[nextIndex]);
            if (nextIndex + 1 < Path.Count)
                DebugInfo += String.Format(" then {0:0.000}", Velocities[nextIndex + 1]);
            vpasterror += verror * (float)gameTime.TotalGameTime.TotalSeconds;
            float vtotalerror = vp * verror + vi * vpasterror;
            if (vtotalerror > 0f)
                brake = vtotalerror;
            else if (vtotalerror < 0f)
                gas = -vtotalerror;

            if (InReverse)
                gas = -gas;

            if (comingToStop && howFar == 1f)
            {
                lastCusp = nextIndex;
                if (nextIndex < Path.Count - 1)
                {
                    NextWaypointIndex++;
                    PrevWaypointIndex++;
                }
                else
                {
                    NextWaypointIndex = PrevWaypointIndex = Path.Count - 1;
                }
                State = ControllerState.Stopped;
                return new CarControls(0f, 1f, steer);
            }

            return new CarControls(gas, brake, steer);
        }

        public CarControls ReverseDrive(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            return ForwardDrive(currentPose, wheelAngle, speed, gameTime);
        }

        public CarControls MissionComplete(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            return new CarControls();
        }

        public override CarControls Update(Pose currentPose, float wheelAngle, float speed, GameTime gameTime)
        {
            lastPose = currentPose;

            if (Path.Count == 0)
                return new CarControls();

            switch (State)
            {
                case ControllerState.MissionStart: return MissionStart();
                case ControllerState.Stopped: return Stopped(currentPose, wheelAngle, speed, gameTime);
                case ControllerState.ForwardDrive: return ForwardDrive(currentPose, wheelAngle, speed, gameTime);
                case ControllerState.ReverseDrive: return ReverseDrive(currentPose, wheelAngle, speed, gameTime);
            }

            return new CarControls();
        }

        public override void Draw(DebugViewXNA draw)
        {
            if (NextWaypointIndex >= Path.Count && FrontPath != null)
            {
                float noop;
            }

            float topSpeed = Velocities == null ? MaxSpeed : Velocities.Max();
            for (int i = 0; i < Path.Count - 1; i++)
            {
                Color color = Color.Lerp(Color.Red, Color.Green, Velocities[i + 1] / topSpeed);
                draw.DrawSegment(Path[i].Position, Path[i + 1].Position, color, 0.04f);
            }

            foreach (int index in stoppingPoints)
            {
                draw.DrawCircle(Path[index].Position, 0.2f, Color.Red);
            }
        } 

        private void localize(Vector2 backAxle, float speed, out int prevIndex, out int nextIndex)
        {
            // Find closest path point to back axle
            float bestd = float.MaxValue;
            int start = Math.Max(lastCusp, Math.Max(0, NextWaypointIndex - 2));
            int end = Math.Min(Path.Count - 1, NextWaypointIndex + 2);
            int besti = start;
            for (int i = start + 1; i < end; i++)
            {
                if (stoppingPoints.Contains(i))
                    break;

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
            else if (besti == lastCusp)
            {
                nextIndex = besti + 1;
                prevIndex = besti;
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

            if (stoppingPoints.Contains(prevIndex) && prevIndex != PrevWaypointIndex)
            {
                prevIndex--;
                nextIndex--;
            }
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
            cte = Math.Abs((r.Y * d.X - r.X * d.Y) / (d.X * d.X + d.Y * d.Y));
            return (r.X * d.X + r.Y * d.Y) / (d.X * d.X + d.Y * d.Y);
        }

        private void updateInfo(float cte, float wheelAngle, float speed, GameTime gameTime)
        {
            CrossTrackError = cte;
            totalCte += cte;
            totalSpeed += speed;

            float lateralAccel = speed * speed / (Car.WHEEL_BASE / (float)Math.Abs(Math.Tan(wheelAngle)));
            if (lateralAccel > maxLateralAccel)
                maxLateralAccel = lateralAccel;
            totalLateralAccel += lateralAccel;

            totalDLateralAccel += (float)Math.Abs(lateralAccel - lastLateralAccel) / (float)gameTime.ElapsedGameTime.TotalSeconds;
            lastLateralAccel = lateralAccel;

            countInfo++;
        }
    }
}
