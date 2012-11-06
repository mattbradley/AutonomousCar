using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XNAGameConsole.Commands
{
    class CustomCommand:IConsoleCommand
    {
        public string Name { get; private set; }
        public string Description { get; private set; }

        private Func<string[], string> action;

        public CustomCommand(string name, Func<string[], string> action, string description)
        {
            Name = name;
            Description = description;
            this.action = action;
        }
        public string Execute(string[] arguments)
        {
            return action(arguments);
        }
    }
}
