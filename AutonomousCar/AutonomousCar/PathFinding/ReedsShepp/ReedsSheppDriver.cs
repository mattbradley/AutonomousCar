using System;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using FarseerPhysics.DebugViews;
using AutonomousCar.Entities;
using C5;

namespace AutonomousCar.PathFinding.ReedsShepp
{
    /// <summary>
    /// The ReedsSheppDriver class is used to expand a ReedsSheppAction. The gear, steer, and length of the action
    /// is used on a starting pose to calculate the resulting ending pose after the action is applied. This class
    /// is also used to convert a ReedsSheppActionSet into a collection of path points.
    /// </summary>
    public class ReedsSheppDriver
    {
        public static DebugViewXNA Debug = null;

        private static Color forwardColor = Color.Purple;
        private static Color backwardColor = Color.DarkOrange;

        private const float maxTurnAngle = 30f / 180f * MathHelper.Pi;
        private const float maxTurnSpeed = 22f / 180f * MathHelper.Pi;
        private float maxCurvature = (float)Math.Tan(maxTurnAngle) / Car.WHEEL_BASE;

        public static void Drive(Pose start, ReedsSheppActionSet actions, float unit)
        {
            Pose current = new Pose(start);
            foreach (ReedsSheppAction action in actions.Actions)
            {
                switch (action.Steer)
                {
                    case Steer.Straight:
                        current = Straight(current, action.Gear, action.Length, unit);
                        break;

                    case Steer.Left:
                        current = TurnLeft(current, action.Gear, action.Length, unit);
                        break;

                    case Steer.Right:
                        current = TurnRight(current, action.Gear, action.Length, unit);
                        break;
                }

                if (Debug != null)
                    current.DrawPose(Debug, 2, Color.LightBlue);
            }
        }

        public static Pose Straight(Pose startPose, Gear gear, float length, float unit) {
            if (gear == Gear.Backward) length = -length;

            length *= unit;

            Pose end = new Pose(
                length * (float)Math.Cos(startPose.Orientation) + startPose.X,
                length * (float)Math.Sin(startPose.Orientation) + startPose.Y,
                startPose.Orientation);

            if (Debug != null)
                Debug.DrawSegment(startPose.Position, end.Position, gear == Gear.Forward ? forwardColor : backwardColor);

            return end;
        }

        public static Pose TurnLeft(Pose startPose, Gear gear, float turnAngle, float unit)
        {
            if (gear == Gear.Backward) turnAngle = -turnAngle;

            float phi = turnAngle / 2;
            float sinPhi = (float)Math.Sin(phi);
            float L = 2 * sinPhi * unit;
            float x = L * (float)Math.Cos(phi);
            float y = L * sinPhi;

            Vector2 pos = new Vector2(x, y);
            pos = Vector2.Transform(pos, Matrix.CreateRotationZ(startPose.Orientation));

            if (Debug != null) {
			    float rotatedTheta = startPose.Orientation + (float)Math.PI / 2;
                float xCenter = startPose.X + unit * (float)Math.Cos(rotatedTheta);
                float yCenter = startPose.Y + unit * (float)Math.Sin(rotatedTheta);
                Debug.DrawArc(new Vector2(xCenter, yCenter), unit, startPose.Orientation - (float)Math.PI / 2, startPose.Orientation + turnAngle - (float)Math.PI / 2, gear == Gear.Forward ? forwardColor : backwardColor);
            }

            return new Pose(pos + startPose.Position, startPose.Orientation + turnAngle);
        }

        public static Pose TurnRight(Pose startPose, Gear gear, float turnAngle, float unit)
        {
            if (gear == Gear.Backward) turnAngle = -turnAngle;

            float phi = turnAngle / 2;
            float sinPhi = (float)Math.Sin(phi);
            float L = 2 * sinPhi * unit;
            float x = L * (float)Math.Cos(phi);
            float y = -L * sinPhi;

            Vector2 pos = new Vector2(x, y);
            pos = Vector2.Transform(pos, Matrix.CreateRotationZ(startPose.Orientation));

            if (Debug != null)
            {
                float rotatedTheta = startPose.Orientation - (float)Math.PI / 2;
                float xCenter = startPose.X + unit * (float)Math.Cos(rotatedTheta);
                float yCenter = startPose.Y + unit * (float)Math.Sin(rotatedTheta);
                Debug.DrawArc(new Vector2(xCenter, yCenter), unit, startPose.Orientation + (float)Math.PI / 2, startPose.Orientation - turnAngle + (float)Math.PI / 2, gear == Gear.Forward ? forwardColor : backwardColor);
            }

            return new Pose(pos + startPose.Position, startPose.Orientation - turnAngle);
        }

        public static ArrayList<Pose> Discretize(Pose start, ReedsSheppActionSet actions, float unit, float maxLength)
        {
            Pose prev = new Pose(start);
            ArrayList<Pose> poses = new ArrayList<Pose>();
            poses.Add(prev);

            foreach (ReedsSheppAction action in actions.Actions)
            {
                int n = (int)Math.Ceiling(action.Length * unit / maxLength);

                if (action.Steer != Steer.Straight)
                {
                    float pieceAngle = action.Length / n;

                    float phi = pieceAngle / 2;
                    float sinPhi = (float)Math.Sin(phi);
                    float L = 2 * sinPhi * unit;
                    float dx = L * (float)Math.Cos(phi);
                    float dy = L * sinPhi;

                    if (action.Steer == Steer.Right)
                    {
                        dy = -dy;
                        pieceAngle = -pieceAngle;
                    }

                    if (action.Gear == Gear.Backward)
                    {
                        dx = -dx;
                        pieceAngle = -pieceAngle;
                    }

                    for (int i = 0; i < n; i++)
                    {
                        Vector2 pos = new Vector2(dx, dy);
                        pos = Vector2.Transform(pos, Matrix.CreateRotationZ(prev.Orientation));
                        prev = new Pose(pos + prev.Position, prev.Orientation + pieceAngle, action.Gear);
                        poses.Add(prev);
                    }
                }
                else
                {
                    float pieceLength = action.Length * unit / n;
                    float dx = pieceLength * (float)Math.Cos(prev.Orientation);
                    float dy = pieceLength * (float)Math.Sin(prev.Orientation);

                    if (action.Gear == Gear.Backward)
                    {
                        dx = -dx;
                        dy = -dy;
                    }

                    for (int i = 0; i < n; i++)
                    {
                        prev = new Pose(dx + prev.X, dy + prev.Y, prev.Orientation, action.Gear);
                        poses.Add(prev);
                    }
                }
            }

            return poses;
        }
    }
}
