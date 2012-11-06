using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XNAGameConsole.Commands
{
    class ClearScreenCommand:IConsoleCommand
    {
        public string Name { get
        {
            return "clear";
        } }
        public string Description { get
        {
            return "Clears the console output";
        } }

        private InputProcessor processor;
        public ClearScreenCommand(InputProcessor processor)
        {
            this.processor = processor;
        }
        public string Execute(string[] arguments)
        {
            processor.Out.Clear();
            return "";
        }
    }
}
