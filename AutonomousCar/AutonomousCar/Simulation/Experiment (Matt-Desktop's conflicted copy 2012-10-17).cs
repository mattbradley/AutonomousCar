using System;
using Microsoft.Xna.Framework;
using AutonomousCar.PathFollowing;
using AutonomousCar.PathFinding;
using AutonomousCar.Entities;
using FarseerPhysics.Dynamics;

namespace AutonomousCar.Simulation
{
    public abstract class Experiment
    {
        protected AutonomousCarSimulation simulation;

        public Experiment(AutonomousCarSimulation simulation)
        {
            this.simulation = simulation;
        }
    }

    public class SmootherExperiment : Experiment
    {
        private string filename;
        private DateTime start;
        private int count;
        private int num;
        private int collisions;
        private bool smoothing;

        public SmootherExperiment(AutonomousCarSimulation simulation, string filename, int num)
            : base(simulation)
        {
            this.filename = filename;
            this.num = num;

            simulation.Collided += new AutonomousCarSimulation.CarCollisionEvent(collided);
            simulation.MissionCompleted += new AutonomousCarSimulation.MissionCompleteEvent(complete);

            Smoother.Anchoring = true;
            smoothing = true;
            count = 0;
            collisions = 0;
            start = DateTime.Now;

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
            {
                DateTime now = DateTime.Now;
                file.WriteLine("--------------------------------");
                file.WriteLine(now.ToString());
                file.WriteLine("num time cte speed lateral maxlateral dlateral\n");
                file.WriteLine("Smoothed (Anchoring):");
            }

            startSim();
        }

        void collided(object sender, EventArgs e)
        {
            collisions++;
            startSim();
        }

        void complete(object sender, ControllerInfo i)
        {
            TimeSpan elapsed = DateTime.Now - start;
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
            {
                file.WriteLine("{0} {1} {2} {3} {4} {5} {6}", count + 1, elapsed.TotalSeconds, i.AverageCTE, i.AverageSpeed, i.AverageLateralAcceleration, i.MaxLateralAcceleration, i.AverageDLateralAcceleration);
            }

            count++;

            if (count < num)
            {
                start = DateTime.Now;
                startSim();
            }
            else if (smoothing && Smoother.Anchoring)
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
                {
                    file.WriteLine("\n" + collisions + " collisions");
                    file.WriteLine("\nSmoother (No Anchoring):");
                }

                count = 0;
                collisions = 0;
                smoothing = true;
                Smoother.Anchoring = false;
            }
            else if (smoothing)
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
                {
                    file.WriteLine("\n" + collisions + " collisions");
                    file.WriteLine("\nUnsmoothed:");
                }

                count = 0;
                collisions = 0;
                smoothing = false;
                Smoother.Anchoring = false;
            }
            else
            {
                using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
                {
                    file.WriteLine("\n" + collisions + " collisions\n");
                }

                simulation.Exit();
            }
        }

        private void startSim()
        {
            simulation.ClearWorld();
            simulation.InitSim(true, smoothing);
        }
    }

    public class ParkingExperiment : Experiment
    {
        private static Vector2 firstRowOffset = new Vector2(10f, 50f);
        private static Vector2 secondRowOffset = new Vector2(40f, 50f);
        private static Vector2 thirdRowOffset = new Vector2(70f, 50f);
        private const float distanceBetweenSpaces = 2.5f;
        private const float distanceBetweenRows = 5f;
        private const int numSpacesInRow = 10;
        private const float carWidth = 2f * Car.HALF_CAR_WIDTH;
        private const float carHeight = 2f * Car.HALF_CAR_LENGTH;

        private World world;
        private string filename;
        private int startIndex;
        private int goalIndex;
        private DateTime start;

        public ParkingExperiment(AutonomousCarSimulation simulation, World world, string filename)
            : base(simulation)
        {
            this.world = world;
            this.filename = filename;

            simulation.MissionCompleted += new AutonomousCarSimulation.MissionCompleteEvent(complete);

            using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
            {
                DateTime now = DateTime.Now;
                file.WriteLine("--------------------------------");
                file.WriteLine(now.ToString());
                file.WriteLine("start goal time perr oerr(rad) oerr(deg)\n");
            }

            startIndex = goalIndex = 0;
            startSim();
        }

        private Environment buildEnvironment(int start, int goal, out Pose startPose, out Pose goalPose)
        {
            startPose = new Pose();
            goalPose = new Pose();

            Environment e = new Environment();
            e.GridWidth = 100;
            e.GridHeight = 100;
            e.GridResolution = 0.5f;

            for (int i = -1; i < numSpacesInRow + 1; i++)
            {
                if (i == start)
                {
                    Vector2 pos = firstRowOffset + new Vector2(0f, distanceBetweenSpaces * i) - new Vector2(Car.WHEEL_POS_LR.X, 0f);
                    startPose = new Pose(pos, MathHelper.Pi);
                }
                else
                {
                    e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, firstRowOffset + new Vector2(0f, distanceBetweenSpaces * i)));
                }

                e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, firstRowOffset - new Vector2(distanceBetweenRows, 0f) + new Vector2(0f, distanceBetweenSpaces * i)));
            }

            for (int i = -1; i < numSpacesInRow + 1; i += i == 0 || i == 6 ? 3 : 1)
            {
                e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, secondRowOffset + new Vector2(0f, distanceBetweenSpaces * i)));
                e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, secondRowOffset - new Vector2(distanceBetweenRows, 0f) + new Vector2(0f, distanceBetweenSpaces * i)));
            }
            /*for (int i = -2 + numSpacesInRow / 2; i < numSpacesInRow / 2 + 2; i++)
            {
                e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, secondRowOffset + new Vector2(0f, distanceBetweenSpaces * i)));
                e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, secondRowOffset - new Vector2(distanceBetweenRows, 0f) + new Vector2(0f, distanceBetweenSpaces * i)));
            }*/

            for (int i = -1; i < numSpacesInRow + 1; i++)
            {

                e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, thirdRowOffset + new Vector2(0f, distanceBetweenSpaces * i)));
                if (i == goal)
                {
                    Vector2 pos = thirdRowOffset - new Vector2(distanceBetweenRows, 0f) + new Vector2(0f, distanceBetweenSpaces * i) - new Vector2(Car.WHEEL_POS_LR.X, 0f);
                    goalPose = new Pose(pos, MathHelper.Pi);
                }
                else
                {
                    e.Obstacles.Add(new BoxObstacle(world, carHeight, carWidth, thirdRowOffset - new Vector2(distanceBetweenRows, 0f) + new Vector2(0f, distanceBetweenSpaces * i)));
                }
            }

            return e;
        }

        void complete(object sender, ControllerInfo i)
        {
            TimeSpan elapsed = DateTime.Now - start;
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(filename, true))
            {
                file.WriteLine("{0} {1} {2} {3} {4} {5}", startIndex, goalIndex, elapsed.TotalSeconds, i.GoalPositionError, i.GoalOrientationError, MathHelper.ToDegrees(i.GoalOrientationError));
            }

            if (++goalIndex >= numSpacesInRow)
            {
                goalIndex = 0;
                if (++startIndex >= numSpacesInRow)
                {
                    simulation.Exit();
                    return;
                }
            }

            startSim();
        }

        private void startSim()
        {
            simulation.ClearWorld();

            start = DateTime.Now;
            Pose startPose, goalPose;
            Mission m = new Mission();
            m.Environment = buildEnvironment(startIndex, goalIndex, out startPose, out goalPose);
            m.Start = startPose;
            m.Goal = goalPose;

            simulation.InitSim(true, true, m);
        }
    }
}