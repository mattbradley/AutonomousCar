using System;
using AutonomousCar.PathFinding;
using AutonomousCar.PathFollowing;
using FarseerPhysics.Controllers;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Joints;
using FarseerPhysics.Factories;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;

namespace AutonomousCar.Entities
{
    /// <summary>
    /// The Wheel class represents a wheel physics object.
    /// </summary>
    public class Wheel
    {
        public const float
            HALF_WHEEL_LENGTH = 0.329f, // meters
            HALF_WHEEL_WIDTH = 0.117f, // meters
            WHEEL_DENSITY = 250f; // kg/m^3

        public Body Body { get; set; }

        public Wheel(World world, Vector2 position, float orientation)
        {
            Body = BodyFactory.CreateRectangle(world, HALF_WHEEL_LENGTH * 2, HALF_WHEEL_WIDTH * 2, WHEEL_DENSITY, position);
            Body.BodyType = BodyType.Dynamic;
            Body.Friction = Car.FRICTION;
            Body.Restitution = Car.RESTITUTION;
            Body.Rotation = orientation;
            //Body.LinearDamping = Car.LINEAR_DAMPING;
            //Body.AngularDamping = Car.ANGULAR_DAMPING;
        }
    }

    /// <summary>
    /// The RevoluteWheel extends the Wheel class to define a wheel that can rotate.
    /// This class is used for the front wheels.
    /// </summary>
    public class RevoluteWheel : Wheel
    {
        public RevoluteJoint Joint { get; set; }

        public RevoluteWheel(World world, Car car, Vector2 position, float orientation)
            : base(world, position, orientation)
        {
            Joint = JointFactory.CreateRevoluteJoint(world, car.Body, Body, Vector2.Zero);
            Joint.MotorEnabled = true;
            Joint.MaxMotorTorque = 100;
            Joint.LimitEnabled = true;
            Joint.LowerLimit = -Car.MAX_WHEEL_DEFLECTION;
            Joint.UpperLimit = Car.MAX_WHEEL_DEFLECTION;
        }
    }

    /// <summary>
    /// The RevoluteWheel extends the Wheel class to define a wheel that cannot rotate.
    /// This class is used for the rear wheels.
    /// </summary>
    public class FixedWheel : Wheel
    {
        public WeldJoint Joint { get; set; }

        public FixedWheel(World world, Car car, Vector2 position, Vector2 anchorPos, float orientation)
            : base(world, position, orientation)
        {
            Joint = JointFactory.CreateWeldJoint(world, car.Body, Body, anchorPos, Vector2.Zero);
        }
    }

    /// <summary>
    /// The WheelSet class is a container for the vehicle's four wheels. It applies a tensor damping controller
    /// to the four wheels to simulate friction and rolling.
    /// </summary>
    public class WheelSet
    {
        public RevoluteWheel LeftFront { get; set; }
        public RevoluteWheel RightFront { get; set; }
        public FixedWheel LeftRear { get; set; }
        public FixedWheel RightRear { get; set; }
        public TensorDampingController Controller { get; set; }

        public WheelSet(World world, Car car, Vector2 position, float orientation)
        {
            Matrix rotation = Matrix.CreateRotationZ(orientation);

            LeftFront = new RevoluteWheel(world, car, position + Vector2.Transform(Car.WHEEL_POS_LF, rotation), orientation);
            RightFront = new RevoluteWheel(world, car, position + Vector2.Transform(Car.WHEEL_POS_RF, rotation), orientation);
            LeftRear = new FixedWheel(world, car, position + Vector2.Transform(Car.WHEEL_POS_LR, rotation), Car.WHEEL_POS_LR, orientation);
            RightRear = new FixedWheel(world, car, position + Vector2.Transform(Car.WHEEL_POS_RR, rotation), Car.WHEEL_POS_RR, orientation);

            Controller = new TensorDampingController();
            Controller.SetAxisAligned(0, 100);
            Controller.AddBody(LeftFront.Body);
            Controller.AddBody(RightFront.Body);
            Controller.AddBody(LeftRear.Body);
            Controller.AddBody(RightRear.Body);

            world.AddController(Controller);
        }
    }

    /// <summary>
    /// The Car class implements a car physics object. The Car has a mass, a turning radius, a size, a engine force, a brake force,
    /// a steer speed, and other constants that control how the vehicle behaves. This class can also
    /// accept car controls (gas, brake, steering) to apply to the car physics body.
    /// </summary>
    public class Car
    {
        public const float
            HALF_CAR_LENGTH = 2.4f, // meters
            HALF_CAR_WIDTH = 0.96f, // meters
            SAFE_RADIUS = 2.58f * 1.1f, // meters, sqrt(halfCarLength^2 + halfCarWidth^2) * safetyFactor
            UNSAFE_RADIUS = 0.96f / 1.1f, // meters, halfCarWidth / safetyFactor
            CHASSIS_DENSITY = 1570f / 9.216f, // kg/m^3
            FRICTION = 0.9f,
            RESTITUTION = 0.1f,
            LINEAR_DAMPING = 0.075f,
            ANGULAR_DAMPING = 0.3f,
            WHEEL_BASE = 2.93f, // meters
            TURNING_RADIUS = 5.0749f, // radians, WHEEL_BASE / tan(30 degrees)

            MAX_ENGINE_FORCE = 5000f, // newtons
            MAX_BRAKE_FORCE = 1570f * 10f, // newtons (1570kg * 10 m/s^2)

            DRAG_COEFF = 0.7f,
            DENSITY_OF_AIR = 1.8580608f, // (kg/m^3)
            FRONTAL_AREA = 1.85f, // m^2
            ROLL_RESIST = 10.157f * 3f,
            MAX_WHEEL_DEFLECTION = 32f / 180f * MathHelper.Pi, // radians
            STEER_SPEED = 1.2f; // Radians per second

        public static Vector2
            WHEEL_POS_LF = new Vector2(1.56f, 0.843f), // meters
            WHEEL_POS_RF = new Vector2(1.56f, -0.843f), // meters
            WHEEL_POS_LR = new Vector2(-1.37f, 0.843f), // meters
            WHEEL_POS_RR = new Vector2(-1.37f, -0.843f); // meters

        private World world;
        private static Model model;

        public Body Body { get; set; }
        public WheelSet Wheels { get; set; }

        public float SpeedMetersPerSecond { get { return Body.LinearVelocity.Length(); } }
        public float SpeedMPH { get { return SpeedMetersPerSecond * 2.23693629f; } }
        public Pose Pose { get { return new Pose(Body.GetWorldPoint(new Vector2(WHEEL_POS_LR.X, 0)), Body.Rotation, WheelAngle); } }
        public float WheelAngle { get { return MathHelper.WrapAngle(Wheels.LeftFront.Body.Rotation - Body.Rotation); } }

        public Car(World world) : this(world, Vector2.Zero, 0f) { }
        public Car(World world, Pose pose) : this(world, pose.Position, pose.Orientation) { }
        public Car(World world, Vector2 position, float orientation)
        {
            float rearAxleDist = -WHEEL_POS_LR.X;
            position += new Vector2((float)Math.Cos(orientation) * rearAxleDist, (float)Math.Sin(orientation) * rearAxleDist);
            Body = BodyFactory.CreateRectangle(world, HALF_CAR_LENGTH * 2, HALF_CAR_WIDTH * 2, CHASSIS_DENSITY, position);
            Body.BodyType = BodyType.Dynamic;
            Body.Friction = FRICTION;
            Body.Restitution = RESTITUTION;
            Body.AngularDamping = ANGULAR_DAMPING;
            Body.Rotation = orientation;

            Wheels = new WheelSet(world, this, position, orientation);
            this.world = world;
        }

        public void LoadContent(ContentManager content)
        {
            model = content.Load<Model>("car");
        }

        public static Vector2 GetFrontAxlePosition(Pose pose)
        {
            return new Vector2(pose.X + (float)Math.Cos(pose.Orientation) * WHEEL_BASE, pose.Y + (float)Math.Sin(pose.Orientation) * WHEEL_BASE);
        }

        public static Vector2 GetFakeFrontAxlePosition(Pose pose)
        {
            return new Vector2(pose.X - (float)Math.Cos(pose.Orientation) * WHEEL_BASE, pose.Y - (float)Math.Sin(pose.Orientation) * WHEEL_BASE);
        }

        public static Vector2 GetCenterPosition(Pose pose)
        {
            return new Vector2(pose.X + (float)Math.Cos(pose.Orientation) * -WHEEL_POS_LR.X, pose.Y + (float)Math.Sin(pose.Orientation) * -WHEEL_POS_LR.X);
        }

        public void Update(CarControls controls)
        {
            Update(controls.Gas, controls.Brake, controls.Steer);
        }

        public void Update(float gasAmount, float brakeAmount, float steerAmount)
        {
            gasAmount = MathHelper.Clamp(gasAmount, -1f, 1f);
            brakeAmount = MathHelper.Clamp(brakeAmount, 0f, 1f);
            steerAmount = MathHelper.Clamp(steerAmount, -1f, 1f);

            float drag = 0.5f * DRAG_COEFF * FRONTAL_AREA * DENSITY_OF_AIR * Body.LinearVelocity.Length();
            float velocity = Body.LinearVelocity.Length();
            Vector2 gasForce = Vector2.Zero;
            Vector2 brakeForce = Vector2.Zero;

            if (brakeAmount > 0)
            {
                if (Body.LinearVelocity.LengthSquared() > 0.1f)
                    brakeForce = MAX_BRAKE_FORCE * brakeAmount * Body.LinearVelocity / velocity;
                else
                    Body.LinearVelocity = Vector2.Zero;
            }
            else
            {
                gasForce = new Vector2((float)Math.Cos(Body.Rotation), (float)Math.Sin(Body.Rotation)) * gasAmount * MAX_ENGINE_FORCE;
            }

            Body.ApplyForce(gasForce - brakeForce - Body.LinearVelocity * (drag + ROLL_RESIST));

            float lAngle = Wheels.LeftFront.Joint.JointAngle;
            float rAngle = Wheels.RightFront.Joint.JointAngle;
            float vSquared = Body.LinearVelocity.LengthSquared();
            Wheels.LeftFront.Joint.MotorSpeed = steerAmount * STEER_SPEED + steerDamping(lAngle, vSquared);
            Wheels.RightFront.Joint.MotorSpeed = steerAmount * STEER_SPEED + steerDamping(rAngle, vSquared);
        }

        public void Draw(Matrix view, Matrix projection)
        {
            //foreach (ModelMesh mesh in model.Meshes)
            for (int i = 0; i < model.Meshes.Count; i++)
            {
                ModelMesh mesh = model.Meshes[i];
                foreach (BasicEffect effect in mesh.Effects)
                {
                    effect.EnableDefaultLighting();
                    effect.GraphicsDevice.DepthStencilState = DepthStencilState.Default;

                    if (i == 1 || i == 3)
                    {
                        effect.World = Matrix.CreateRotationY(WheelAngle) *
                            mesh.ParentBone.Transform *
                            Matrix.CreateRotationZ(Body.Rotation) *
                            Matrix.CreateTranslation(new Vector3(Body.Position, 0f));
                    }
                    else
                    {
                        effect.World = mesh.ParentBone.Transform *
                            Matrix.CreateRotationZ(Body.Rotation) *
                            Matrix.CreateTranslation(new Vector3(Body.Position, 0f));
                    }
                    effect.View = view;
                    effect.Projection = projection;
                }

                mesh.Draw();
            }
        }

        public void Destroy()
        {
            world.RemoveJoint(Wheels.LeftFront.Joint);
            world.RemoveJoint(Wheels.RightFront.Joint);
            world.RemoveJoint(Wheels.LeftRear.Joint);
            world.RemoveJoint(Wheels.RightRear.Joint);

            world.RemoveBody(Wheels.LeftFront.Body);
            world.RemoveBody(Wheels.RightFront.Body);
            world.RemoveBody(Wheels.LeftRear.Body);
            world.RemoveBody(Wheels.RightRear.Body);

            world.RemoveBody(Body);
        }

        private float steerDamping(float deflection, float velocitySquared) {
			return -deflection / MAX_WHEEL_DEFLECTION * velocitySquared * 0.01f;
		}
    }
}