using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XNAGameConsole.Commands
{
    class HelpCommand : IConsoleCommand
    {
        public string Name
        {
            get { return "help"; }
        }

        public string Description
        {
            get { return "Displays the list of commands"; }
        }

        public string Execute(string[] arguments)
        {
            if (arguments != null && arguments.Length >= 1)
            {
                var command = GameConsoleOptions.Commands.Where(c => c.Name != null && c.Name == arguments[0]).FirstOrDefault();
                if (command != null)
                {
                    return String.Format("{0}: {1}\n", command.Name, command.Description);
                }
                return "ERROR: Invalid command '" + arguments[0] + "'";
            }
            var help = new StringBuilder();
            GameConsoleOptions.Commands.Sort(new CommandComparer());
            foreach (var command in GameConsoleOptions.Commands)
            {
                help.Append(String.Format("{0}: {1}\n", command.Name, command.Description));
            }
            return help.ToString();
        }
    }
}
