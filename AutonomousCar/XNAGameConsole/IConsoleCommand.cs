using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XNAGameConsole
{
    public interface IConsoleCommand
    {
        /// <summary>
        /// The name of the command; the command will be invoked through this name
        /// </summary>
        string Name { get; }

        /// <summary>
        /// The description that is displayed with the 'help' command
        /// </summary>
        string Description { get; }

        /// <summary>
        /// The action of the command.  The return string value is used as output in the console 
        /// </summary>
        /// <param name="arguments"></param>
        /// <returns></returns>
        string Execute(string[] arguments);
    }
}
