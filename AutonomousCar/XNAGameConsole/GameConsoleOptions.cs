using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Graphics;
using Microsoft.Xna.Framework.Input;

namespace XNAGameConsole
{
    public class GameConsoleOptions
    {
        public Keys ToggleKey { get; set; }
        public Color BackgroundColor { get; set; }
        public Color FontColor
        {
            set
            {
                BufferColor = PastCommandColor = PastCommandOutputColor = PromptColor = CursorColor = value;
            }
        }
        public Color BufferColor { get; set; }
        public Color PastCommandColor { get; set; }
        public Color PastCommandOutputColor { get; set; }
        public Color PromptColor { get; set; }
        public Color CursorColor { get; set; }
        public float AnimationSpeed { get; set; }
        public float CursorBlinkSpeed { get; set; }
        public int Height { get; set; }
        public string Prompt { get; set; }
        public char Cursor { get; set; }
        public int Padding { get; set; }
        public int Margin { get; set; }
        public bool OpenOnWrite { get; set; }
        public SpriteFont Font { get; set; }

        internal Texture2D RoundedCorner { get; set; }

        internal static GameConsoleOptions Options { get; set; }
        internal static List<IConsoleCommand> Commands { get; set; }

        public GameConsoleOptions()
        {
            //Default options
            ToggleKey = Keys.OemTilde;
            BackgroundColor = new Color(0, 0, 0, 125);
            FontColor = Color.White;
            AnimationSpeed = 1;
            CursorBlinkSpeed = 0.5f;
            Height = 300;
            Prompt = "$";
            Cursor = '_';
            Padding = 30;
            Margin = 30;
            OpenOnWrite = true;
        }

    }
}
