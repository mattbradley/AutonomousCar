using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XNAGameConsole
{
    class CommandHistory:List<string>
    {
        public int Index { get; private set; }

        public void Reset()
        {
            Index = Count;
        }

        public string Next()
        {
            return Count == 0 ? "" : Index + 1 > Count - 1 ? this[Count - 1] : this[++Index];
        }

        public string Previous()
        {
            return Count == 0 ? "" : Index - 1 < 0 ? this[0] : this[--Index];
        }

        public new void Add(string command)
        {
            var parts = command.Split('\n');
            foreach (var part in parts)
            {
                if (part != "")
                {
                    base.Add(part);
                }
            }
            Reset();
        }
    }
}
