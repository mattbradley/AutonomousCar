using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace XNAGameConsole.KeyboardCapture
{
    class KeyEventArgs : EventArgs
    {
        public KeyEventArgs( Keys keyCode )
        {
            KeyCode = keyCode;
        }

        public Keys KeyCode { get; private set; }
    }
    
}
