using System;

namespace RSVisualizer
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// The main entry point for the application.
        /// </summary>
        static void Main(string[] args)
        {
            using (RSVisualizer game = new RSVisualizer())
            {
                game.Run();
            }
        }
    }
#endif
}

