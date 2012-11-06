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

namespace RSVisualizer
{
    public class RSVisualizer : Microsoft.Xna.Framework.Game
    {
        private const int WINDOWED_WIDTH = 1024;
        private const int WINDOWED_HEIGHT = 768;

        GraphicsDeviceManager graphics;
        SpriteBatch spriteBatch;
        Camera camera;
        ArrayList<ArrayList<Pose>> paths;
        ArrayList<ReedsSheppActionSet> actions;
        DebugViewXNA debug;
        World world;
        Pose goal = new Pose();
        float orientation = 0f;
        int count = 0;
        DateTime lastCount = DateTime.Now + TimeSpan.FromSeconds(10);
        KeyboardState prevKeyboard, currKeyboard;

        float square = 100f;
        int cells = 10;
        float cellSize;
        int numCells;
        float maxLength;

        bool drawGrid = true,
            drawFill = false;

        public RSVisualizer()
        {
            graphics = new GraphicsDeviceManager(this);
            Content.RootDirectory = "Content";
        }

        protected override void Initialize()
        {
            graphics.PreferMultiSampling = true;
            graphics.PreferredBackBufferWidth = WINDOWED_WIDTH;
            graphics.PreferredBackBufferHeight = WINDOWED_HEIGHT;
            graphics.IsFullScreen = false;
            graphics.ApplyChanges();

            spriteBatch = new SpriteBatch(GraphicsDevice);
            camera = new Camera(MathHelper.PiOver4, GraphicsDevice.Viewport.AspectRatio, 0.1f, 1000f);
            camera.Position = new Vector3(0f, 0f, 140f);
            world = new World(Vector2.Zero);
            numCells = (cells + 1) * (cells + 1);
            cellSize = square / cells;
            calcRS();

            base.Initialize();
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

        private void calcRS()
        {
            maxLength = float.MinValue;
            paths = new ArrayList<ArrayList<Pose>>();
            actions = new ArrayList<ReedsSheppActionSet>();
            for (int x = -cells / 2; x <= cells / 2; x++)
                for (int y = -cells / 2; y <= cells / 2; y++)
                {
                    Pose start = new Pose(x * cellSize, y * cellSize, orientation);
                    ReedsSheppActionSet actionSet = ReedsSheppSolver.Solve(start, goal, VehicleModel.TurnRadius);
                    actionSet.Length = actionSet.CalculateCost(VehicleModel.TurnRadius, HybridAStar.reverseFactor, HybridAStar.switchPenalty);
                    if (actionSet.Length > maxLength)
                        maxLength = actionSet.Length;
                    actions.Add(actionSet);
                    paths.Add(ReedsSheppDriver.Discretize(start, actionSet, VehicleModel.TurnRadius, 1f));
                }
        }

        protected override void LoadContent()
        {
            spriteBatch = new SpriteBatch(GraphicsDevice);
            debug = new DebugViewXNA(world);
            debug.LoadContent(GraphicsDevice, Content);
        }

        private bool keyPressed(Keys key)
        {
            return currKeyboard.IsKeyDown(key) && prevKeyboard.IsKeyUp(key);
        }

        protected override void Update(GameTime gameTime)
        {
            currKeyboard = Keyboard.GetState();

            if (Keyboard.GetState().IsKeyDown(Keys.Escape))
                this.Exit();

            camera.HandleInput();

            if (keyPressed(Keys.F11))
                toggleFullscreen();

            DateTime now = DateTime.Now;
            if (count >= numCells)
            {
                if (now - lastCount >= TimeSpan.FromSeconds(2f))
                {
                    orientation += MathHelper.PiOver4;
                    while (orientation < 0f) orientation += MathHelper.TwoPi;
                    while (orientation >= MathHelper.TwoPi) orientation -= MathHelper.TwoPi;
                    calcRS();
                    count = 0;
                }
            }
            else if (now - lastCount >= TimeSpan.FromSeconds(0.05))
            {
                count++;
                lastCount = now;
                drawGrid = false;
            }

            prevKeyboard = currKeyboard;

            base.Update(gameTime);
        }

        protected override void Draw(GameTime gameTime)
        {
            GraphicsDevice.Clear(Color.White);//new Color(0x22, 0x22, 0x22));

            Matrix p = camera.Projection;
            Matrix v = camera.View;
            debug.BeginCustomDraw(ref p, ref v);

            if (drawGrid || true)
            {
                for (int x = -cells / 2; x <= cells / 2 + 1; x++)
                {
                    float pos = x * cellSize - cellSize / 2f;
                    debug.DrawSegment(new Vector2(pos, -square / 2f - cellSize / 2f), new Vector2(pos, square / 2f + cellSize / 2f), Color.Gray);
                }

                for (int y = -cells / 2; y <= cells / 2 + 1; y++)
                {
                    float pos = y * cellSize - cellSize / 2f;
                    debug.DrawSegment(new Vector2(-square / 2f - cellSize / 2f, pos), new Vector2(square / 2f + cellSize / 2f, pos), Color.Gray);
                }
            }

            for (int n = 0; n < paths.Count; n++)
            {
                if (n >= count) break;
                float val = actions[n].Length / maxLength;
                Color c = Color.Lerp(Color.Green, Color.Red, val);
                ArrayList<Pose> poses = paths[n];
                for (int i = 0; i < poses.Count - 1; i++)
                    debug.DrawSegment(poses[i].Position, poses[i + 1].Position, c, 0.01f);
            }

            debug.DrawSolidCircle(goal.Position, 2f, Color.Blue);
            debug.DrawSegment(goal.Position, Vector2.Transform(goal.Position + 5f * Vector2.UnitX, Matrix.CreateRotationZ(goal.Orientation)), Color.Blue, 0.2f);

            debug.EndCustomDraw();
            base.Draw(gameTime);
        }
    }
}
