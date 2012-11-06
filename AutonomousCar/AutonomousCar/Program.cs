using System;
using AutonomousCar.Simulation;

namespace AutonomousCar
{
#if WINDOWS || XBOX
    static class Program
    {
        /// <summary>
        /// This is the main entry point for the simulation. See the AutonomousCarSimulation for the main driver class.
        /// </summary>
        static void Main(string[] args)
        {
            using (AutonomousCarSimulation sim = new AutonomousCarSimulation())
            {
                sim.Run();
            }
        }
    }
#endif
}

