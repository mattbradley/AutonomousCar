using System;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using AutonomousCar.Simulation;

namespace AutonomousCar.Helpers
{
    /// <summary>
    /// The Dashboard class provides a graphical interface for the vehicle's current controls (gas, brake, steering).
    /// It also gives the vehicle's current speed.
    /// </summary>
    public class Dashboard
    {
        private AutonomousCarSimulation simulation;
        private Texture2D wheelTexture;
        private Texture2D blankTexture;
        private SpriteFont font;
        private SpriteBatch batch;

        private float wheelAngle;
        private float gas;
        private float brake;
        private float speed;

        public Dashboard(AutonomousCarSimulation simulation)
        {
            this.simulation = simulation;
            wheelAngle = 0f;
        }

        public void LoadContent(ContentManager content)
        {
            wheelTexture = content.Load<Texture2D>("wheel");
            blankTexture = content.Load<Texture2D>("blank");
            font = content.Load<SpriteFont>("font");
            batch = new SpriteBatch(simulation.GraphicsDevice);
        }

        public void Update(float wheelAngle, float gas, float brake, float speed)
        {
            this.wheelAngle = wheelAngle;
            this.gas = MathHelper.Clamp(gas, 0f, 1f);
            this.brake = MathHelper.Clamp(brake, 0f, 1f);
            this.speed = speed;
        }

        public void Draw(GameTime gameTime)
        {
            int sw = simulation.GraphicsDevice.Viewport.Width, sh = simulation.GraphicsDevice.Viewport.Height;

            Rectangle wheelRect = new Rectangle(sw - 120, sh - 140, 200, 200);

            bool showGas = gas >= brake;
            int gasBrake = (int)Math.Round(100 * (showGas ? gas : brake));
            Rectangle gasBrakeRect = new Rectangle(sw - 210 + (100 - gasBrake) / 2, sh - 30, gasBrake, 20);
            Rectangle gasBrakeBackdropRect = new Rectangle(sw - 210, sh - 30, 100, 20);
            Rectangle speedBackdrop = new Rectangle(sw - 100, sh - 30, 65, 20);

            string speedString = String.Format("{0:0.0} mph", Math.Round(speed, 1));
            Vector2 speedSize = font.MeasureString(speedString);

            batch.Begin();
            batch.Draw(wheelTexture, wheelRect, null, Color.White, wheelAngle * -14f, new Vector2(252, 252), SpriteEffects.None, 0f);
            batch.Draw(blankTexture, gasBrakeBackdropRect, Color.Gray);
            batch.Draw(blankTexture, speedBackdrop, Color.Gray);
            batch.Draw(blankTexture, gasBrakeRect, showGas ? Color.LightGreen : Color.Red);
            batch.DrawString(font, speedString, new Vector2(sw - 100 + (60 - speedSize.X), sh - 28), Color.White);
            batch.End();
        }
    }
}