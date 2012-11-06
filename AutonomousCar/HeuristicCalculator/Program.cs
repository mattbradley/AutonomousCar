using System;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using AutonomousCar.PathFinding.Algorithms;

namespace HeuristicCalculatorProgram
{
    class Program
    {
        private static int count = 0;

        static void Main(string[] args)
        {
            HeuristicCalculator.CellCalculated += new HeuristicCalculator.CellCalculatedEvent(cellCalculated);
            NonholonomiHeuristicInfo info = HeuristicCalculator.Calculate(150f, 0.75f, 72);
            
            Stream stream = File.Open(@"c:\rsheurnorev.dat", FileMode.Create);
            (new BinaryFormatter()).Serialize(stream, info);
            stream.Close();
        }

        static private void cellCalculated(object sender, CalculatedArgs e)
        {
            if (count++ % 10000 == 0)
                Console.WriteLine(e.Progress * 100 + "%");
        }
    }
}
