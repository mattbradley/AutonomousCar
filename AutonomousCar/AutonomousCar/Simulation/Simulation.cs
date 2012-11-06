using System;
using System.Threading;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Audio;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.GamerServices;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;
using Microsoft.Xna.Framework.Media;
using FarseerPhysics.Collision.Shapes;
using FarseerPhysics.Common;
using FarseerPhysics.Dynamics;
using FarseerPhysics.Dynamics.Contacts;
using FarseerPhysics.Factories;
using FarseerPhysics.DebugViews;
using AutonomousCar.PathFinding;
using AutonomousCar.Entities;
using AutonomousCar.Helpers;
using AutonomousCar.PathFinding.ReedsShepp;
using AutonomousCar.PathFinding.Algorithms;
using AutonomousCar.PathFollowing;
using C5;
using XNAGameConsole;

namespace AutonomousCar.Simulation
{
    /// <summary>
    /// This AutonomousCarSimulation class is the main driver for the autonomous car simulation.
    /// It handles loading missions, running the simulator, updating the physics engine, handling input, and drawing the scene.
    /// </summary>
    public class AutonomousCarSimulation : Game
    {
        private string missionFile;
        private string expFile = "null.txt";

        private const float CAMERA_FOV = 90.0f;
        private const float CAMERA_ZNEAR = 0.01f;
        private const float CAMERA_ZFAR = 1000.0f;
        private const float CAMERA_OFFSET = 0.0f;

        private const int WINDOWED_WIDTH = 1024;
        private const int WINDOWED_HEIGHT = 768;

        private static Color LIGHT = Color.White;
        private static Color DARK = new Color(0x22, 0x22, 0x22);

        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        SpriteFont font;
        DebugViewXNA debug;
        BasicEffect basicEffect;
        World world;
        Car car;
        Camera camera;
        Dashboard dashboard;
        Random r;
        KeyboardState currKeyboard, prevKeyboard;
        GameConsole console;
        Thread bg;
        bool gridDone,
            pathDone,
            pathSmoothDone,
            pathSearching,
            pathSearchingDone,
            run,
            isCollided;

        float gas = 0, steer = 0, brake = 0;

        float scale = 4.8f,
            xOffset = 75f,
            yOffset = 75f;

        bool rotateCar = false,
            autoDrive = true,
            watchSearch = false,
            drawGrid = false,
            drawVoro = false,
            drawHeur = false,
            drawSearch = false,
            drawPath = false,
            drawFrontPath = false,
            drawSmoothedPath = true,
            drawController = false,
            drawStart = false,
            drawGoal = true,
            drawCurrent = true,
            drawCar = true,
            drawDebugData = true,
            smoothing = true,
            backgroundLight = true,
            showDebugInfo = false,
            showDashboard = true,
            paused = false;

        int cameraView = 1;

        ObstacleGrid grid;
        Pose startPose, goalPose;
        ArrayList<Pose> poses, smoothedPath;
        HybridAStarResults astar;
        StanleyFSMController controller;
        CarControls currentControls;

        public delegate void CarCollisionEvent(object sender, EventArgs e);
        public delegate void MissionCompleteEvent(object sender, ControllerInfo e);
        public event CarCollisionEvent Collided;
        public event MissionCompleteEvent MissionCompleted;

        public AutonomousCarSimulation()
        {
            graphics = new GraphicsDeviceManager(this);

            Content.RootDirectory = "Content";
            r = new Random();

            Components.Add(new FrameRateCounter(this));
        }

        protected override void Initialize()
        {
            graphics.PreferMultiSampling = true;
            graphics.PreferredBackBufferWidth = WINDOWED_WIDTH;
            graphics.PreferredBackBufferHeight = WINDOWED_HEIGHT;
            graphics.IsFullScreen = false;
            graphics.ApplyChanges();

            spriteBatch = new SpriteBatch(GraphicsDevice);
            console = new GameConsole(this, spriteBatch, new GameConsoleOptions()
            {
                OpenOnWrite = false,
                Prompt = ">",
                Height = 500,
                ToggleKey = Keys.Escape
            });

            console.AddCommand("load",
                args =>
                {
                    missionFile = "../../../../../missions/" + args[0] + ".json";
                    ClearWorld();
                    InitSim(false);
                    return "Mission loaded.";
                }, "Load a mission file");

            console.AddCommand("experiment",
                args =>
                {
                    if (args[0] == "random")
                        new SmootherExperiment(this, expFile, 100);
                    else if (args[0] == "parking")
                        new ParkingExperiment(this, world, expFile);
                    else
                        return "Experiment not found.";

                    return "Experiment started.";
                }, "Run an experiment");

            console.AddCommand("antialias",
                args =>
                {
                    graphics.PreferMultiSampling = !graphics.PreferMultiSampling;
                    graphics.ApplyChanges();
                    return "Antialias " + (graphics.PreferMultiSampling ? "on" : "off");
                }, "Toggles antialias");

            console.AddCommand("debug",
                 args =>
                 {
                     showDebugInfo = !showDebugInfo;
                     return "Debug " + (showDebugInfo ? "on" : "off");
                 }, "Toggles the debug info");

            console.AddCommand("dashboard",
                 args =>
                 {
                     showDashboard = !showDashboard;
                     return "Dashboard " + (showDashboard ? "on" : "off");
                 }, "Toggles the dashboard");

            console.AddCommand("param",
                args =>
                {
                    if (args.Length < 1) return "";

                    switch (args[0].ToLower())
                    {
                        case "smoother":
                            return Smoother.HandleParamCommand(args);
                        default:
                            return "";
                    }
                }, "Prints or updates parameters");

            console.AddCommand("smooth",
                args =>
                {
                    if (pathSmoothDone)
                    {
                        bg = new Thread(() =>
                            {
                                DateTime now = DateTime.Now;
                                smoothedPath = Smoother.Smooth(astar.Path, grid);
                                int numUnsafe = Smoother.UnsafeIndices != null ? Smoother.UnsafeIndices.Count : 0;
                                console.WriteLine("Smoothing Time: " + Math.Round((DateTime.Now - now).TotalMilliseconds) + " ms (" + Smoother.NumIterations + " iterations, " + Smoother.Change + "m, " + numUnsafe + " unsafe points)");
                                controller = new StanleyFSMController(smoothedPath, goalPose);
                            });
                        bg.IsBackground = true;
                        bg.Priority = ThreadPriority.Lowest;
                        bg.Start();
                    }

                    return "";
                }, "Smooths the path using the current parameters");

            console.AddCommand("smoothing",
                args =>
                {
                    smoothing = !smoothing;
                    return "Smoothing " + (smoothing ? "on" : "off");
                }, "Toggles smoothing"
            );

            dashboard = new Dashboard(this);
            world = new World(Vector2.Zero);
            car = new Car(world, new Pose());
            camera = new Camera(MathHelper.PiOver4, GraphicsDevice.Viewport.AspectRatio, 0.1f, 1000f);
            camera.Position = new Vector3(75f, 75f, 180f);

            base.Initialize();
        }

        public void InitSim(bool autoStart)
        {
            InitSim(autoStart, smoothing, MissionLoader.LoadFromJson(missionFile, world));
        }

        public void InitSim(bool autoStart, bool smoothingOn)
        {
            InitSim(autoStart, smoothingOn, MissionLoader.LoadFromJson(missionFile, world));
        }

        public void InitSim(bool autoStart, bool smoothingOn, string missionFilename)
        {
            InitSim(autoStart, smoothingOn, MissionLoader.LoadFromJson(missionFilename, world));
        }

        public void InitSim(bool autoStart, bool smoothingOn, Mission mission)
        {
            gridDone = false;
            pathDone = false;
            pathSmoothDone = false;
            pathSearching = false;
            pathSearchingDone = false;
            autoDrive = true;
            run = false;
            isCollided = false;
            currentControls = new CarControls(0f, 0f, 0f);

            if (camera == null)
            {
                camera = new Camera(MathHelper.PiOver4, GraphicsDevice.Viewport.AspectRatio, 0.1f, 1000f);
                camera.Position = new Vector3(75f, 75f, 180f);
            }
                        
            startPose = mission.Start;
            goalPose = mission.Goal;
            HybridAStar.Epsilon = mission.AStarEpsilon;
            HybridAStar.GridResolution = mission.AStarGridResolution;
            HybridAStar.SafetyFactor = 1.5f;
            HybridAStar.Reset();

            car = new Car(world, startPose);
            grid = new ObstacleGrid(world, mission.Environment);

            car.Body.OnCollision += new OnCollisionEventHandler(OnCollision);

            gridDone = false;
            pathDone = false;
            pathSearchingDone = false;
            bg = new Thread(() =>
            {
                DateTime now = DateTime.Now;
                grid.BuildGVD();
                console.WriteLine("GVD Generation Time: " + Math.Round((DateTime.Now - now).TotalMilliseconds) + " ms");
                gridDone = true;

                now = DateTime.Now;
                pathSearching = true;
                astar = HybridAStar.FindPath(grid, startPose, goalPose);
                TimeSpan astarTime = DateTime.Now - now;
                poses = astar.Path;

                pathSearching = false;
                pathSearchingDone = true;

                if (astar.Path.Count > 0)
                    pathDone = true;

                now = DateTime.Now;
                if (smoothingOn)
                    smoothedPath = Smoother.Smooth(astar.Path, grid);
                else
                    smoothedPath = astar.Path;
                TimeSpan smoothingTime = DateTime.Now - now;

                int numUnsafe = Smoother.UnsafeIndices != null ? Smoother.UnsafeIndices.Count : 0;

                console.WriteLine("A*: Total Planning Time: " + Math.Round((astarTime + smoothingTime).TotalMilliseconds) + " ms");
                console.WriteLine("         Heuristic Time: " + Math.Round(astar.HeuristicInitTime.TotalMilliseconds) + " ms");
                console.WriteLine("         Searching Time: " + Math.Round((astarTime - astar.HeuristicInitTime).TotalMilliseconds) + " ms");
                console.WriteLine("         Smoothing Time: " + Math.Round(smoothingTime.TotalMilliseconds) + " ms (" + Smoother.NumIterations + " iterations, " + Smoother.Change + "m, " + numUnsafe + " unsafe points)");
                console.WriteLine("    " + astar.Discovered.Count + " nodes discovered");
                console.WriteLine("    " + astar.Expanded.Count + " nodes expanded");

                controller = new StanleyFSMController(smoothedPath, goalPose);

                pathSmoothDone = true;
                if (autoStart)
                    run = true;
            });
            bg.IsBackground = true;
            bg.Priority = ThreadPriority.Lowest;
            bg.Start();
        }

        public void ClearWorld()
        {
            if (world != null)
                world.Clear();
        }

        public bool OnCollision(Fixture fixtureA, Fixture fixtureB, Contact contact)
        {
            isCollided = true;
            return true;
        }

        protected override void LoadContent()
        {
            RasterizerState rs = new RasterizerState();
            rs.CullMode = CullMode.None;
            GraphicsDevice.RasterizerState = rs;
            font = Content.Load<SpriteFont>("font");

            car.LoadContent(Content);
            basicEffect = new BasicEffect(GraphicsDevice)
            {
                TextureEnabled = true,
                VertexColorEnabled = true,
            };

            debug = new DebugViewXNA(world);
            debug.LoadContent(GraphicsDevice, Content);

            dashboard.LoadContent(Content);
        }

        private void toggleFullscreen()
        {
            if (graphics.IsFullScreen)
            {
                graphics.PreferredBackBufferWidth = WINDOWED_WIDTH;
                graphics.PreferredBackBufferHeight = WINDOWED_HEIGHT;
                graphics.IsFullScreen = false;
            }
            else
            {
                graphics.PreferredBackBufferWidth = GraphicsDevice.DisplayMode.Width;
                graphics.PreferredBackBufferHeight = GraphicsDevice.DisplayMode.Height;
                graphics.IsFullScreen = true;
            }

            graphics.ApplyChanges();
            camera.AspectRatio = (float)graphics.PreferredBackBufferWidth / graphics.PreferredBackBufferHeight;
        }

        protected override void Update(GameTime gameTime)
        {
            currKeyboard = Keyboard.GetState();

            if (currKeyboard.IsKeyDown(Keys.Escape) && currKeyboard.IsKeyDown(Keys.LeftShift))
                this.Exit();

            if (!console.Opened)
            {
                camera.HandleInput();

                // View options
                if (keyPressed(Keys.F11))
                    toggleFullscreen();
                if (keyPressed(Keys.F12))
                    backgroundLight = !backgroundLight;
                if (keyPressed(Keys.C))
                    drawCar = !drawCar;
                if (keyPressed(Keys.R))
                    rotateCar = !rotateCar;
                if (keyPressed(Keys.G))
                {
                    drawGrid = !drawGrid;
                    drawVoro = false;
                    drawHeur = false;
                }
                if (keyPressed(Keys.V))
                {
                    drawVoro = !drawVoro;
                    drawGrid = false;
                    drawHeur = false;
                }
                if (keyPressed(Keys.H))
                {
                    drawHeur = !drawHeur;
                    drawVoro = false;
                    drawGrid = false;
                }
                if (keyPressed(Keys.B))
                    if (currKeyboard.IsKeyDown(Keys.LeftShift) || currKeyboard.IsKeyDown(Keys.RightShift))
                        watchSearch = !watchSearch;
                    else
                        drawSearch = !drawSearch;
                if (keyPressed(Keys.P))
                    drawPath = !drawPath;
                if (keyPressed(Keys.L))
                {
                    drawSmoothedPath = !drawSmoothedPath;
                    drawController = false;
                }
                if (keyPressed(Keys.O))
                {
                    drawController = !drawController;
                    drawSmoothedPath = false;
                }
                if (keyPressed(Keys.F))
                    drawFrontPath = !drawFrontPath;
                if (keyPressed(Keys.Home))
                    drawStart = !drawStart;
                if (keyPressed(Keys.End))
                    drawGoal = !drawGoal;
                if (keyPressed(Keys.Scroll))
                    drawCurrent = !drawCurrent;
                if (keyPressed(Keys.N))
                    drawDebugData = !drawDebugData;

                if (keyPressed(Keys.OemComma))
                    HybridAStar.Delay = HybridAStar.Delay == 0 ? 1 : 0;
                if (keyPressed(Keys.OemPeriod))
                    Smoother.Delay = Smoother.Delay == 0 ? 10 : 0;

                // Zoom
                if (currKeyboard.IsKeyDown(Keys.OemPlus))
                    scale *= 1.02f;
                if (currKeyboard.IsKeyDown(Keys.OemMinus))
                    scale *= 0.98f;

                // Translate
                if (currKeyboard.IsKeyDown(Keys.Up))
                    yOffset += 5f / scale;
                if (currKeyboard.IsKeyDown(Keys.Down))
                    yOffset -= 5f / scale;
                if (currKeyboard.IsKeyDown(Keys.Right))
                    xOffset += 5f / scale;
                if (currKeyboard.IsKeyDown(Keys.Left))
                    xOffset -= 5f / scale;

                if (currKeyboard.IsKeyDown(Keys.W))
                    gas = 1;
                else if (currKeyboard.IsKeyDown(Keys.S))
                    gas = -1;
                else
                    gas = 0;

                if (currKeyboard.IsKeyDown(Keys.D))
                    steer = -1;
                else if (currKeyboard.IsKeyDown(Keys.A))
                    steer = 1;
                else
                    steer = 0;

                if (currKeyboard.IsKeyDown(Keys.Space))
                    brake = 1;
                else
                    brake = 0;

                if (gas != 0 || brake != 0 || steer != 0)
                    autoDrive = false;

                if (currKeyboard.IsKeyDown(Keys.Enter) && pathSmoothDone)
                    run = true;

                // Camera views
                if (keyPressed(Keys.D1))
                    cameraView = 1;
                if (keyPressed(Keys.D2))
                    cameraView = 2;
                if (keyPressed(Keys.D3))
                    cameraView = 3;
                if (keyPressed(Keys.D4))
                    cameraView = 4;
                if (keyPressed(Keys.D5))
                    cameraView = 5;

                if (keyPressed(Keys.D0))
                {
                    ClearWorld();
                    InitSim(false);
                }

                if (keyPressed(Keys.Back))
                    paused = !paused;
            }

            prevKeyboard = currKeyboard;

            if (!paused)
            {
                if (autoDrive)
                {
                    if (run)
                    {
                        currentControls = controller.Update(car.Pose, car.WheelAngle, car.SpeedMetersPerSecond, gameTime);
                        car.Update(currentControls);
                    }
                }
                else
                {
                    currentControls = new CarControls(gas, brake, steer);
                    car.Update(new CarControls(gas, brake, steer));
                }

                if (run && controller.State == StanleyFSMController.ControllerState.MissionComplete && MissionCompleted != null)
                    MissionCompleted(this, controller.Info);

                world.Step((float)gameTime.ElapsedGameTime.TotalSeconds);
                dashboard.Update(car.WheelAngle, currentControls.Gas, currentControls.Brake, car.SpeedMPH);

                if (isCollided && Collided != null)
                    Collided(this, null);
            }

            base.Update(gameTime);
        }

        private bool keyPressed(Keys key)
        {
            return currKeyboard.IsKeyDown(key) && prevKeyboard.IsKeyUp(key);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(backgroundLight ? LIGHT : DARK);

            Matrix p = camera.Projection;
            Matrix v = camera.View;

            if (cameraView == 2)
            {
                float orientation = controller != null && controller.InReverse ? car.Pose.Orientation + MathHelper.Pi : car.Pose.Orientation;
                Vector2 position = Car.GetCenterPosition(car.Pose);
                Vector2 headingVector = new Vector2((float)Math.Cos(orientation), (float)Math.Sin(orientation));
                float offset = 14f;
                float height = 10f;
                v = Matrix.CreateLookAt(new Vector3(position + -offset * headingVector, height), new Vector3(position + offset * headingVector, 0f), Vector3.UnitZ);
            }
            else if (cameraView == 3)
            {
                float or = car.Pose.Orientation + MathHelper.ToRadians(120f);
                Vector2 headingVector = new Vector2((float)Math.Cos(or), (float)Math.Sin(or));
                v = Matrix.CreateLookAt(new Vector3(car.Pose.Position + -10f * headingVector, 5f), new Vector3(Car.GetCenterPosition(car.Pose), 0f), Vector3.UnitZ);
            }
            else if (cameraView == 4)
            {
                float or = goalPose.Orientation;
                Vector2 headingVector = new Vector2((float)Math.Cos(or), (float)Math.Sin(or));
                v = Matrix.CreateLookAt(new Vector3(goalPose.Position + 10f * headingVector, 20f), new Vector3(Car.GetCenterPosition(car.Pose), 0f), Vector3.UnitZ);
            }
            else if (cameraView == 5)
            {
                v = Matrix.CreateLookAt(new Vector3(Car.GetCenterPosition(car.Pose), 20f), new Vector3(Car.GetCenterPosition(car.Pose), 0f), Vector3.UnitY);
                if (rotateCar)
                    v *= Matrix.CreateRotationZ(-car.Body.Rotation + MathHelper.PiOver2);
            }

            if (gridDone && (drawGrid || drawVoro || drawHeur))
            {
                basicEffect.World = Matrix.Identity;
                basicEffect.View = v;
                basicEffect.Projection = p;
                spriteBatch.Begin(0, null, SamplerState.PointClamp, DepthStencilState.DepthRead, RasterizerState.CullNone, basicEffect);

                if (drawHeur)
                    HybridAStar.Draw(spriteBatch);
                else
                    grid.Draw(spriteBatch, drawVoro);

                spriteBatch.End();
            }

            debug.BeginCustomDraw(ref p, ref v);

            debug.DrawSolidCircle(Car.GetFrontAxlePosition(car.Pose), 0.2f, Color.Green);

            if ((pathSearching && watchSearch || pathSearchingDone && drawSearch) && HybridAStar.Expanded != null && HybridAStar.Expanded.Count > 0)
            {
                LinkedList<HybridAStar.Node> expanded = new LinkedList<HybridAStar.Node>();
                lock (HybridAStar.Expanded)
                {
                    expanded.AddAll(HybridAStar.Expanded);
                }

                float pathLength = expanded.Last.f;
                foreach (HybridAStar.Node n in expanded)
                {
                    if (pathDone && n.rsIndex >= 0)
                    {
                        for (int i = n.rsIndex; i < poses.Count - 1; i++)
                            debug.DrawSegment(poses[i].Position, poses[i + 1].Position, Color.Purple, 0.02f);
                    }
                    else if (n.from != null)
                    {
                        Color color;
                        if (n.cell.rev == 0)
                            color = Color.Lerp(Color.Orange, Color.Green, n.g / pathLength);
                        else
                            color = Color.Lerp(Color.Blue, Color.Cyan, n.g / pathLength);

                        debug.DrawSegment(n.from.pose.Position, n.pose.Position, color, 0.02f);
                    }
                }
            }

            if (pathDone)
            {
                if (drawPath)
                {
                    for (int i = 0; i < poses.Count - 1; i++)
                    {
                        Color c = poses[i].Gear == Gear.Forward ? Color.Blue : Color.Red;
                        debug.DrawSegment(poses[i].Position, poses[i + 1].Position, c, 0.04f);
                        debug.DrawPoint(poses[i].Position, 0.1f, c * 0.5f);
                    }
                }
            }

            if (pathSearchingDone && !pathSmoothDone && (drawSmoothedPath || drawController))
                Smoother.Draw(debug);

            if (pathSmoothDone)
            {
                if (drawFrontPath && controller.FrontPath != null)
                {
                    int num = controller.FrontPath.Count;
                    for (int i = 0; i < num - 1; i++)
                    {
                        if (controller.FrontPath[i].Gear == Gear.Forward)
                            debug.DrawSegment(controller.FrontPath[i].Position, controller.FrontPath[i + 1].Position, Color.DarkOrange);
                        else
                            debug.DrawSegment(controller.ReverseFrontPath[i].Position, controller.ReverseFrontPath[i + 1].Position, Color.Cyan);
                    }
                }

                if (drawSmoothedPath)
                {
                    for (int i = 0; i < smoothedPath.Count - 1; i++)
                    {
                        debug.DrawSegment(smoothedPath[i].Position, smoothedPath[i + 1].Position, smoothedPath[i].Gear == Gear.Forward ? Color.DarkGreen : Color.Red, 0.04f);
                        //if (Smoother.UnsafeIndices != null && Smoother.UnsafeIndices.Contains(i))
                            //debug.DrawCircle(smoothedPath[i].Position, 0.2f, Color.Orange);
                    }
                }

                if (drawController)
                    controller.Draw(debug);
            }

            if (drawStart)
                startPose.DrawPose(debug, new Color(0f, 1f, 0f, 0.9f), 1.1f);
            if (drawGoal)
                goalPose.DrawPose(debug, new Color(1f, 0f, 0f, 0.9f), 1.1f);

            if (drawCurrent)
            {
                if (pathSmoothDone && controller.State != StanleyFSMController.ControllerState.MissionComplete)
                {
                    debug.DrawCircle(controller.ClosestPoint, 0.1f, controller.InReverse ? Color.Aqua : Color.Orange);
                    if (controller.InReverse) debug.DrawCircle(controller.FakeFrontAxle, 0.2f, Color.Aqua);
                }

                car.Pose.DrawPose(debug, 0.2f, Color.Red);
            }

            debug.EndCustomDraw();

            if (drawDebugData)
                debug.RenderDebugData(ref p, ref v);

            if (drawCar)
                car.Draw(v, p);

            if (showDebugInfo)
            {
                string info = String.Format("Speed: {0:0.0}", Math.Round(car.SpeedMPH, 1));
                if (pathSmoothDone)
                {
                    info += String.Format("\nGas: {0:0.00}", Math.Round(currentControls.Gas * 100f, 2));
                    info += String.Format("\nBrake: {0:0.00}", Math.Round(currentControls.Brake * 100f, 2));
                    info += String.Format("\nCTE: {0:0.0000}", Math.Round(controller.CrossTrackError, 4));
                    info += "\n" + controller.State.ToString();
                    info += "\n" + controller.DebugInfo;
                }
                spriteBatch.Begin();
                spriteBatch.DrawString(font, info, new Vector2(8, 4), !backgroundLight ? LIGHT : DARK);
                spriteBatch.End();
            }

            if (showDashboard)
                dashboard.Draw(gameTime);

            base.Draw(gameTime);
        }
    }
}