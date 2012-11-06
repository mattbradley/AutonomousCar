using System;
using System.Collections.Generic;
using System.Linq;
using System.Resources;
using System.Text;
using System.Threading;
using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Content;
using Microsoft.Xna.Framework.Graphics;
using XNAGameConsole.Commands;

namespace XNAGameConsole
{
    public class GameConsole
    {
        public GameConsoleOptions Options { get { return GameConsoleOptions.Options; } }
        public List<IConsoleCommand> Commands { get { return GameConsoleOptions.Commands; } }
        public bool Enabled { get; set; }

        /// <summary>
        /// Indicates whether the console is currently opened
        /// </summary>
        public bool Opened { get { return console.IsOpen; } }

        private readonly GameConsoleComponent console;

        public GameConsole(Game game, SpriteBatch spriteBatch) : this(game, spriteBatch, new IConsoleCommand[0], new GameConsoleOptions()) { }
        public GameConsole(Game game, SpriteBatch spriteBatch, GameConsoleOptions options) : this(game, spriteBatch, new IConsoleCommand[0],options){}
        public GameConsole(Game game, SpriteBatch spriteBatch, IEnumerable<IConsoleCommand> commands) :this(game,spriteBatch,commands,new GameConsoleOptions()){}
        public GameConsole(Game game, SpriteBatch spriteBatch, IEnumerable<IConsoleCommand> commands, GameConsoleOptions options) 
        {
            var content = new ResourceContentManager(game.Services, Resource1.ResourceManager);
            if (options.Font == null)
            {
                options.Font = content.Load<SpriteFont>("ConsoleFont");
            }
            options.RoundedCorner = content.Load<Texture2D>("roundedCorner");
            GameConsoleOptions.Options = options;
            GameConsoleOptions.Commands = commands.ToList();
            Enabled = true;
            console = new GameConsoleComponent(this, game, spriteBatch);
            game.Services.AddService(typeof(GameConsole), this);
            game.Components.Add(console); 
        }

        /// <summary>
        /// Write directly to the output stream of the console
        /// </summary>
        /// <param name="text"></param>
        public void WriteLine(string text)
        {
            console.WriteLine(text);
        }

        /// <summary>
        /// Adds a new command to the console
        /// </summary>
        /// <param name="commands"></param>
        public void AddCommand(params IConsoleCommand[] commands)
        {
            Commands.AddRange(commands);
        }

        /// <summary>
        /// Adds a new command to the console
        /// </summary>
        /// <param name="name">Name of the command</param>
        /// <param name="action"></param>
        public void AddCommand(string name, Func<string[], string> action)
        {
            AddCommand(name, action,"");
        }

        /// <summary>
        /// Adds a new command to the console
        /// </summary>
        /// <param name="name">Name of the command</param>
        /// <param name="action"></param>
        /// <param name="description"></param>
        public void AddCommand(string name, Func<string[], string> action, string description)
        {
            Commands.Add(new CustomCommand(name,action,description));
        }
    }
}
