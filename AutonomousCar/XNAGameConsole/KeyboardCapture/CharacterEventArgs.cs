using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace XNAGameConsole.KeyboardCapture
{
    class CharacterEventArgs : EventArgs
    {
        private readonly char character;
        private readonly int lParam;

        public CharacterEventArgs( char character, int lParam )
        {
            this.character = character;
            this.lParam = lParam;
        }

        public char Character
        {
            get { return character; }
        }

        public int Param
        {
            get { return lParam; }
        }

        public int RepeatCount
        {
            get { return lParam & 0xffff; }
        }

        public bool ExtendedKey
        {
            get { return ( lParam & ( 1 << 24 ) ) > 0; }
        }

        public bool AltPressed
        {
            get { return ( lParam & ( 1 << 29 ) ) > 0; }
        }

        public bool PreviousState
        {
            get { return ( lParam & ( 1 << 30 ) ) > 0; }
        }

        public bool TransitionState
        {
            get { return ( lParam & ( 1 << 31 ) ) > 0; }
        }
    }
}
