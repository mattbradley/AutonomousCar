using System;
using AutonomousCar.Entities;
using FarseerPhysics.DebugViews;
using Microsoft.Xna.Framework;

namespace AutonomousCar.PathFinding
{
    /// <summary>
    /// The Pose struct represents a vehicle state, including x and y position and orientation.
    /// A vehicles wheel angle and gear are also stored here for various components of the simulation.
    /// </summary>
    public struct Pose
    {
        public Vector2 Position;
        public float Orientation;
        public float WheelAngle;
        public float X { get { return Position.X; } set { Position = new Vector2(value, Position.Y); } }
        public float Y { get { return Position.Y; } set { Position = new Vector2(Position.X, value); } }
        public Gear Gear;
        
        public Pose(Pose copy) : this(copy.Position, copy.Orientation, copy.WheelAngle) { }
        public Pose(Vector2 position) : this(position, 0f, 0f) { }
        public Pose(Vector2 position, float orientation) : this(position, orientation, 0f) { }
        public Pose(Vector2 position, float orientation, Gear gear) : this(position, orientation, 0f, gear) { }
        public Pose(float x, float y) : this(new Vector2(x, y), 0f, 0f) { }
        public Pose(float x, float y, float orientation) : this(new Vector2(x, y), orientation, 0f) { }
        public Pose(float x, float y, float orientation, Gear gear) : this(new Vector2(x, y), orientation, 0f, gear) { }
        public Pose(float x, float y, float orientation, float wheelAngle) : this(new Vector2(x, y), orientation, wheelAngle) { }
        public Pose(Vector2 position, float orientation, float wheelAngle) : this(position, orientation, wheelAngle, Gear.Forward) { }

        public Pose(Vector2 position, float orientation, float wheelAngle, Gear gear)
        {
            Position = position;
            while (orientation < 0f) orientation += MathHelper.TwoPi;
            while (orientation >= MathHelper.TwoPi) orientation -= MathHelper.TwoPi;
            Orientation = orientation;
            WheelAngle = wheelAngle;
            Gear = gear;
        }

        public void DrawPose(DebugViewXNA debugDraw, float radius, Color color)
        {
            debugDraw.DrawSolidCircle(Position, radius, color);
            debugDraw.DrawSegment(Position, Position + radius * Vector2.Transform(Vector2.UnitX, Matrix.CreateRotationZ(Orientation)), color);
        }

        public void DrawPose(DebugViewXNA debugDraw, Color color)
        {
            DrawPose(debugDraw, color, 1.0f);
        }

        public void DrawPose(DebugViewXNA debugDraw, Color color, float scale)
        {
            Vector2 pos = Car.GetCenterPosition(this);
            debugDraw.DrawBox(Car.HALF_CAR_LENGTH * scale, Car.HALF_CAR_WIDTH * scale, pos, Orientation, color);
        }

        public static bool operator==(Pose me, Pose other) {
            return me.Position == other.Position && me.Orientation == other.Orientation;
        }

        public static bool operator !=(Pose me, Pose other)
        {
            return me.Position != other.Position || me.Orientation != other.Orientation;
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
}
