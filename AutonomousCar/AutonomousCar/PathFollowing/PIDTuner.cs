using System;
using Microsoft.Xna.Framework;

namespace AutonomousCar.PathFollowing
{
    /// <summary>
    /// The PIDTuner class was used to iteratively tune the gains of the PIDController. This class is no longer used.
    /// </summary>
    public class PIDTuner
    {
        private PIDController controller;
        private float totalcte;
        private int numSteps;
        private int stepsPerIteration;
        private float[] gains;
        private float[] deltas;
        private int currGain;
        private bool checkNeg = false;

        public float PGain { get { return gains[0]; } }
        public float DGain { get { return gains[1]; } }
        public float PGainDelta { get { return deltas[0]; } }
        public float DGainDelta { get { return deltas[1]; } }
        public float BestAvgCTE { get; private set; }

        public PIDTuner(float initialPGain, float initialDGain, int stepsPerIteration, PIDController controller)
        {
            gains = new float[2];
            deltas = new float[] { 0.1f, 0.1f };
            gains[0] = controller.PGain = initialPGain;
            gains[1] = controller.DGain = initialDGain;
            this.stepsPerIteration = stepsPerIteration;
            this.controller = controller;

            numSteps = 0;
            totalcte = 0f;
            BestAvgCTE = float.MaxValue;
            currGain = 0;
        }

        public bool Update(GameTime gameTime)
        {
            totalcte += controller.CrossTrackError;
            numSteps++;

            if (numSteps >= stepsPerIteration)
            {
                float currcte = totalcte / numSteps;
                if (currcte < BestAvgCTE)
                {
                    BestAvgCTE = currcte;
                    deltas[currGain] *= 1.1f;
                }
                else if (checkNeg)
                {
                    gains[currGain] += deltas[currGain];
                    updateGains();
                    deltas[currGain] *= 0.9f;
                    checkNeg = false;
                }
                else
                {
                    gains[currGain] -= 2 * deltas[currGain];
                    updateGains();
                    checkNeg = true;
                }

                if (!checkNeg)
                {
                    currGain = (currGain + 1) % 2;
                    gains[currGain] += deltas[currGain];
                    updateGains();
                }

                totalcte = 0f;
                numSteps = 0;

                return true;
            }

            return false;
        }

        private void updateGains()
        {
            controller.PGain = gains[0];
            controller.DGain = gains[1];
        }
    }
}
