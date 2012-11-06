using System;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;
using C5;

namespace AutonomousCar.PathFinding.ReedsShepp
{
    /// <summary>
    /// The ReedsSheppSolver class takes a start and goal pose and generates the optimal set of Reeds-Shepp actions
    /// that can be used to move a vehicle from start to goal obeying the turning radius contraints of the vehicle.
    /// This class implements dozens of trigonometric equations described by Reeds and Shepp in their paper:
    /// "Optimal paths for a car that goes both forwards and backwards".
    /// </summary>
    public class ReedsSheppSolver
    {
        public const int NumPathWords = 48;

        public static ReedsSheppActionSet Solve(Pose start, Pose goal, float unit)
        {
            // Translate the goal so that the start position is at the origin
            Pose newGoal = new Pose((goal.Position - start.Position) / unit, MathHelper.WrapAngle(goal.Orientation - start.Orientation));

            // Rotate the goal so that the start orientation is 0
            newGoal.Position = Vector2.Transform(newGoal.Position, Matrix.CreateRotationZ(-start.Orientation));

            float bestPathLength = float.PositiveInfinity;
            PathWords bestWord = 0;
            float t = 0, u = 0, v = 0, _t, _u, _v;

            for (int w = 0; w < NumPathWords; w++) {
                PathWords word = (PathWords)w;
                float potentialLength = CalculatePathLength(newGoal, word, out _t, out _u, out _v);

                if (potentialLength < bestPathLength)
                {
                    bestPathLength = potentialLength;
                    bestWord = word;
                    t = _t;
                    u = _u;
                    v = _v;
                }
            }

            if (float.IsPositiveInfinity(bestPathLength))
                return new ReedsSheppActionSet(float.PositiveInfinity);

            return GetPath(bestWord, t, u, v);
        }

        public static float CalculatePathLength(Pose goal, PathWords word, out float t, out float u, out float v)
        {
            switch (word)
            {
                // Reeds-Shepp 8.1: CSC, same turn
                case PathWords.LfSfLf: return LfSfLf(goal, out t, out u, out v);
                case PathWords.LbSbLb: return LfSfLf(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfSfRf: return LfSfLf(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbSbRb: return LfSfLf(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.2: CSC, different turn
                case PathWords.LfSfRf: return LfSfRf(goal, out t, out u, out v);
                case PathWords.LbSbRb: return LfSfRf(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfSfLf: return LfSfRf(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbSbLb: return LfSfRf(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.3: C|C|C
                case PathWords.LfRbLf: return LfRbLf(goal, out t, out u, out v);
                case PathWords.LbRfLb: return LfRbLf(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLbRf: return LfRbLf(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLfRb: return LfRbLf(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.4: C|CC
                case PathWords.LfRbLb: return LfRbLb(goal, out t, out u, out v);
                case PathWords.LbRfLf: return LfRbLb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLbRb: return LfRbLb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLfRf: return LfRbLb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.4: CC|C
                case PathWords.LfRfLb: return LfRfLb(goal, out t, out u, out v);
                case PathWords.LbRbLf: return LfRfLb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLfRb: return LfRfLb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLbRf: return LfRfLb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.7: CCu|CuC
                case PathWords.LfRufLubRb: return LfRufLubRb(goal, out t, out u, out v);
                case PathWords.LbRubLufRf: return LfRufLubRb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLufRubLb: return LfRufLubRb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLubRufLf: return LfRufLubRb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.8: C|CuCu|C
                case PathWords.LfRubLubRf: return LfRubLubRf(goal, out t, out u, out v);
                case PathWords.LbRufLufRb: return LfRubLubRf(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLubRubLf: return LfRubLubRf(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLufRufLb: return LfRubLubRf(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
                case PathWords.LfRbpi2SbLb: return LfRbpi2SbLb(goal, out t, out u, out v);
                case PathWords.LbRfpi2SfLf: return LfRbpi2SbLb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLbpi2SbRb: return LfRbpi2SbLb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLfpi2SfRf: return LfRbpi2SbLb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
                case PathWords.LfRbpi2SbRb: return LfRbpi2SbRb(goal, out t, out u, out v);
                case PathWords.LbRfpi2SfRf: return LfRbpi2SbLb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLbpi2SbLb: return LfRbpi2SbLb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLfpi2SfLf: return LfRbpi2SbLb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
                case PathWords.LfSfRfpi2Lb: return LfSfRfpi2Lb(goal, out t, out u, out v);
                case PathWords.LbSbRbpi2Lf: return LfSfRfpi2Lb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfSfLfpi2Rb: return LfSfRfpi2Lb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbSbLbpi2Rf: return LfSfRfpi2Lb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
                case PathWords.LfSfLfpi2Rb: return LfSfLfpi2Rb(goal, out t, out u, out v);
                case PathWords.LbSbLbpi2Rf: return LfSfLfpi2Rb(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfSfRfpi2Lb: return LfSfLfpi2Rb(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbSbRbpi2Lf: return LfSfLfpi2Rb(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);

                // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
                case PathWords.LfRbpi2SbLbpi2Rf: return LfRbpi2SbLbpi2Rf(goal, out t, out u, out v);
                case PathWords.LbRfpi2SfLfpi2Rb: return LfRbpi2SbLbpi2Rf(new Pose(-goal.X, goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RfLbpi2SbRbpi2Lf: return LfRbpi2SbLbpi2Rf(new Pose(goal.X, -goal.Y, -goal.Orientation), out t, out u, out v);
                case PathWords.RbLfpi2SfRfpi2Lb: return LfRbpi2SbLbpi2Rf(new Pose(-goal.X, -goal.Y, goal.Orientation), out t, out u, out v);                
            }

            t = 0; u = 0; v = 0;
            return float.PositiveInfinity;
        }

        public static ReedsSheppActionSet GetPath(PathWords word, float t, float u, float v)
        {
            switch (word)
            {
                // Reeds-Shepp 8.1: CSC, same turn
                case PathWords.LfSfLf: return LfSfLfpath(t, u, v);
                case PathWords.LbSbLb: return timeflipTransform(LfSfLfpath(t, u, v));
                case PathWords.RfSfRf: return reflectTransform(LfSfLfpath(t, u, v));
                case PathWords.RbSbRb: return reflectTransform(timeflipTransform(LfSfLfpath(t, u, v)));

                // Reeds-Shepp 8.2: CSC, different turn
                case PathWords.LfSfRf: return LfSfRfpath(t, u, v);
                case PathWords.LbSbRb: return timeflipTransform(LfSfRfpath(t, u, v));
                case PathWords.RfSfLf: return reflectTransform(LfSfRfpath(t, u, v));
                case PathWords.RbSbLb: return reflectTransform(timeflipTransform(LfSfRfpath(t, u, v)));

                // Reeds-Shepp 8.3: C|C|C
                case PathWords.LfRbLf: return LfRbLfpath(t, u, v);
                case PathWords.LbRfLb: return timeflipTransform(LfRbLfpath(t, u, v));
                case PathWords.RfLbRf: return reflectTransform(LfRbLfpath(t, u, v));
                case PathWords.RbLfRb: return reflectTransform(timeflipTransform(LfRbLfpath(t, u, v)));

                // Reeds-Shepp 8.4: C|CC
                case PathWords.LfRbLb: return LfRbLbpath(t, u, v);
                case PathWords.LbRfLf: return timeflipTransform(LfRbLbpath(t, u, v));
                case PathWords.RfLbRb: return reflectTransform(LfRbLbpath(t, u, v));
                case PathWords.RbLfRf: return reflectTransform(timeflipTransform(LfRbLbpath(t, u, v)));

                // Reeds-Shepp 8.4: CC|C
                case PathWords.LfRfLb: return LfRfLbpath(t, u, v);
                case PathWords.LbRbLf: return timeflipTransform(LfRfLbpath(t, u, v));
                case PathWords.RfLfRb: return reflectTransform(LfRfLbpath(t, u, v));
                case PathWords.RbLbRf: return reflectTransform(timeflipTransform(LfRfLbpath(t, u, v)));

                // Reeds-Shepp 8.7: CCu|CuC
                case PathWords.LfRufLubRb: return LfRufLubRbpath(t, u, v);
                case PathWords.LbRubLufRf: return timeflipTransform(LfRufLubRbpath(t, u, v));
                case PathWords.RfLufRubLb: return reflectTransform(LfRufLubRbpath(t, u, v));
                case PathWords.RbLubRufLf: return reflectTransform(timeflipTransform(LfRufLubRbpath(t, u, v)));

                // Reeds-Shepp 8.8: C|CuCu|C
                case PathWords.LfRubLubRf: return LfRubLubRfpath(t, u, v);
                case PathWords.LbRufLufRb: return timeflipTransform(LfRubLubRfpath(t, u, v));
                case PathWords.RfLubRubLf: return reflectTransform(LfRubLubRfpath(t, u, v));
                case PathWords.RbLufRufLb: return reflectTransform(timeflipTransform(LfRubLubRfpath(t, u, v)));

                // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
                case PathWords.LfRbpi2SbLb: return LfRbpi2SbLbpath(t, u, v);
                case PathWords.LbRfpi2SfLf: return timeflipTransform(LfRbpi2SbLbpath(t, u, v));
                case PathWords.RfLbpi2SbRb: return reflectTransform(LfRbpi2SbLbpath(t, u, v));
                case PathWords.RbLfpi2SfRf: return reflectTransform(timeflipTransform(LfRbpi2SbLbpath(t, u, v)));

                // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
                case PathWords.LfRbpi2SbRb: return LfRbpi2SbRbpath(t, u, v);
                case PathWords.LbRfpi2SfRf: return timeflipTransform(LfRbpi2SbRbpath(t, u, v));
                case PathWords.RfLbpi2SbLb: return reflectTransform(LfRbpi2SbRbpath(t, u, v));
                case PathWords.RbLfpi2SfLf: return reflectTransform(timeflipTransform(LfRbpi2SbRbpath(t, u, v)));

                // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
                case PathWords.LfSfRfpi2Lb: return LfSfRfpi2Lbpath(t, u, v);
                case PathWords.LbSbRbpi2Lf: return timeflipTransform(LfSfRfpi2Lbpath(t, u, v));
                case PathWords.RfSfLfpi2Rb: return reflectTransform(LfSfRfpi2Lbpath(t, u, v));
                case PathWords.RbSbLbpi2Rf: return reflectTransform(timeflipTransform(LfSfRfpi2Lbpath(t, u, v)));

                // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
                case PathWords.LfSfLfpi2Rb: return LfSfLfpi2Rbpath(t, u, v);
                case PathWords.LbSbLbpi2Rf: return timeflipTransform(LfSfLfpi2Rbpath(t, u, v));
                case PathWords.RfSfRfpi2Lb: return reflectTransform(LfSfLfpi2Rbpath(t, u, v));
                case PathWords.RbSbRbpi2Lf: return reflectTransform(timeflipTransform(LfSfLfpi2Rbpath(t, u, v)));

                // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
                case PathWords.LfRbpi2SbLbpi2Rf: return LfRbpi2SbLbpi2Rfpath(t, u, v);
                case PathWords.LbRfpi2SfLfpi2Rb: return timeflipTransform(LfRbpi2SbLbpi2Rfpath(t, u, v));
                case PathWords.RfLbpi2SbRbpi2Lf: return reflectTransform(LfRbpi2SbLbpi2Rfpath(t, u, v));
                case PathWords.RbLfpi2SfRfpi2Lb: return reflectTransform(timeflipTransform(LfRbpi2SbLbpi2Rfpath(t, u, v)));
            }

            return new ReedsSheppActionSet(float.PositiveInfinity);
        }

        private static ReedsSheppActionSet timeflipTransform(ReedsSheppActionSet actions)
        {
            foreach (ReedsSheppAction a in actions.Actions)
                a.Gear = a.Gear == Gear.Backward ? Gear.Forward : Gear.Backward;
            return actions;
        }

        private static ReedsSheppActionSet reflectTransform(ReedsSheppActionSet actions)
        {
            foreach (ReedsSheppAction a in actions.Actions)
                if (a.Steer == Steer.Left)
                    a.Steer = Steer.Right;
                else if (a.Steer == Steer.Right)
                    a.Steer = Steer.Left;
            return actions;
        }

        private static float LfSfLf(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.1
            t = 0; u = 0; v = 0;

            float x = goal.X - (float)Math.Sin(goal.Orientation);
            float y = goal.Y - 1 + (float)Math.Cos(goal.Orientation);

            u = (float)Math.Sqrt(x * x + y * y);
            t = (float)Math.Atan2(y, x);
            v = MathHelper.WrapAngle(goal.Orientation - t);

            if (isInvalidAngle(t) || isInvalidAngle(v))
                return float.PositiveInfinity;

            return t + u + v;
        }

        private static ReedsSheppActionSet LfSfLfpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Straight, Gear.Forward, u);
            actions.AddAction(Steer.Left, Gear.Forward, v);
            return actions;
        }

        private static float LfSfRf(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.2
            t = 0; u = 0; v = 0;

            float x = goal.X + (float)Math.Sin(goal.Orientation);
            float y = goal.Y - 1 - (float)Math.Cos(goal.Orientation);

            float u1squared = x * x + y * y;
            float t1 = (float)Math.Atan2(y, x);

            if (u1squared < 4)
                return float.PositiveInfinity;

            u = (float)Math.Sqrt(u1squared - 4);
            float phi = (float)Math.Atan2(2, u);
            t = MathHelper.WrapAngle(t1 + phi);
            v = MathHelper.WrapAngle(t - goal.Orientation);

            if (isInvalidAngle(t) || isInvalidAngle(v))
                return float.PositiveInfinity;

            return t + u + v;
        }

        private static ReedsSheppActionSet LfSfRfpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Straight, Gear.Forward, u);
            actions.AddAction(Steer.Right, Gear.Forward, v);
            return actions;
        }

        private static float LfRbLf(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.3
            // Uses a modified formula adapted from the c_c_c function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X - (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 + (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 > 4)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);
            float alpha = (float)Math.Acos(u1 / 4f);
            t = mod2PI(MathHelper.PiOver2 + alpha + phi);
            u = mod2PI(MathHelper.Pi - 2 * alpha);
            v = mod2PI(goal.Orientation - t - u);

            if (isInvalidAngle(t) || isInvalidAngle(u) || isInvalidAngle(v))
                return float.PositiveInfinity;

            return t + u + v;
        }

        private static ReedsSheppActionSet LfRbLfpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Backward, u);
            actions.AddAction(Steer.Left, Gear.Forward, v);
            return actions;
        }

        private static float LfRbLb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.4
            // Uses a modified formula adapted from the c_cc function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X - (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 + (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 > 4)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);
            float alpha = (float)Math.Acos(u1 / 4f);
            t = mod2PI(MathHelper.PiOver2 + alpha + phi);
            u = mod2PI(MathHelper.Pi - 2 * alpha);
            v = mod2PI(t + u - goal.Orientation);

            return t + u + v;
        }

        private static ReedsSheppActionSet LfRbLbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Backward, u);
            actions.AddAction(Steer.Left, Gear.Backward, v);
            return actions;
        }

        private static float LfRfLb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.4
            // Uses a modified formula adapted from the cc_c function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X - (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 + (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 > 4)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);
            u = (float)Math.Acos((8 - u1 * u1) / 8f);
            float va = (float)Math.Sin(u);
            float alpha = (float)Math.Asin(2 * va / u1);
            t = mod2PI(MathHelper.PiOver2 - alpha + phi);
            v = mod2PI(t - u - goal.Orientation);

            return t + u + v;
        }

        private static ReedsSheppActionSet LfRfLbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Forward, u);
            actions.AddAction(Steer.Left, Gear.Backward, v);
            return actions;
        }

        private static float LfRufLubRb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.7
            // Uses a modified formula adapted from the ccu_cuc function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X + (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 - (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 > 4)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);

            if (u1 > 2)
            {
                float alpha = (float)Math.Acos(u1 / 4 - 0.5);
                t = mod2PI(MathHelper.PiOver2 + phi - alpha);
                u = mod2PI(MathHelper.Pi - alpha);
                v = mod2PI(goal.Orientation - t + 2 * u);
            }
            else
            {
                float alpha = (float)Math.Acos(u1 / 4 + 0.5);
                t = mod2PI(MathHelper.PiOver2 + phi + alpha);
                u = mod2PI(alpha);
                v = mod2PI(goal.Orientation - t + 2 * u);
            }

            return t + u + u + v;
        }

        private static ReedsSheppActionSet LfRufLubRbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Forward, u);
            actions.AddAction(Steer.Left, Gear.Backward, u);
            actions.AddAction(Steer.Right, Gear.Backward, v);
            return actions;
        }

        private static float LfRubLubRf(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.8
            // Uses a modified formula adapted from the c_cucu_c function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X + (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 - (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 > 6)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);
            float va1 = 1.25f - u1 * u1 / 16;
            if (va1 < 0 || va1 > 1)
                return float.PositiveInfinity;

            u = (float)Math.Acos(va1);
            float va2 = (float)Math.Sin(u);
            float alpha = (float)Math.Asin(2 * va2 / u1);
            t = mod2PI(MathHelper.PiOver2 + phi + alpha);
            v = mod2PI(t - goal.Orientation);

            return t + u + u + v;
        }

        private static ReedsSheppActionSet LfRubLubRfpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Backward, u);
            actions.AddAction(Steer.Left, Gear.Backward, u);
            actions.AddAction(Steer.Right, Gear.Forward, v);
            return actions;
        }

        private static float LfRbpi2SbLb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.9
            // Uses a modified formula adapted from the c_c2sca function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X - (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 + (float)Math.Cos(goal.Orientation);

            float u1squared = xi * xi + eta * eta;
            if (u1squared < 4)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);

            u = (float)Math.Sqrt(u1squared - 4) - 2;
            if (u < 0)
                return float.PositiveInfinity;

            float alpha = (float)Math.Atan2(2, u + 2);
            t = mod2PI(MathHelper.PiOver2 + phi + alpha);
            v = mod2PI(t + MathHelper.PiOver2 - goal.Orientation);

            return t + MathHelper.PiOver2 + u + v;
        }

        private static ReedsSheppActionSet LfRbpi2SbLbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Backward, MathHelper.PiOver2);
            actions.AddAction(Steer.Straight, Gear.Backward, u);
            actions.AddAction(Steer.Left, Gear.Backward, v);
            return actions;
        }

        private static float LfRbpi2SbRb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.10
            // Uses a modified formula adapted from the c_c2scb function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X + (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 - (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 < 2)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);

            t = mod2PI(MathHelper.PiOver2 + phi);
            u = u1 - 2;
            v = mod2PI(goal.Orientation - t - MathHelper.PiOver2);

            return t + MathHelper.PiOver2 + u + v;
        }

        private static ReedsSheppActionSet LfRbpi2SbRbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Backward, MathHelper.PiOver2);
            actions.AddAction(Steer.Straight, Gear.Backward, u);
            actions.AddAction(Steer.Right, Gear.Backward, v);
            return actions;
        }

        private static float LfSfRfpi2Lb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.9 (reversed)
            // Uses a modified formula adapted from the csc2_ca function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X - (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 + (float)Math.Cos(goal.Orientation);

            float u1squared = xi * xi + eta * eta;
            if (u1squared < 4)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);

            u = (float)Math.Sqrt(u1squared - 4) - 2;
            if (u < 0)
                return float.PositiveInfinity;

            float alpha = (float)Math.Atan2(u + 2, 2);
            t = mod2PI(MathHelper.PiOver2 + phi - alpha);
            v = mod2PI(t - MathHelper.PiOver2 - goal.Orientation);

            return t + u + MathHelper.PiOver2 + v;
        }

        private static ReedsSheppActionSet LfSfRfpi2Lbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Straight, Gear.Forward, u);
            actions.AddAction(Steer.Right, Gear.Forward, MathHelper.PiOver2);
            actions.AddAction(Steer.Left, Gear.Backward, v);
            return actions;
        }

        private static float LfSfLfpi2Rb(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.10 (reversed)
            // Uses a modified formula adapted from the csc2_cb function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X + (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 - (float)Math.Cos(goal.Orientation);

            float u1 = (float)Math.Sqrt(xi * xi + eta * eta);
            if (u1 < 2)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);

            t = mod2PI(phi);
            u = u1 - 2;
            v = mod2PI(-t - MathHelper.PiOver2 + goal.Orientation);

            return t + u + MathHelper.PiOver2 + v;
        }

        private static ReedsSheppActionSet LfSfLfpi2Rbpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Straight, Gear.Forward, u);
            actions.AddAction(Steer.Left, Gear.Forward, MathHelper.PiOver2);
            actions.AddAction(Steer.Right, Gear.Backward, v);
            return actions;
        }

        private static float LfRbpi2SbLbpi2Rf(Pose goal, out float t, out float u, out float v)
        {
            // Reeds-Shepp 8.11
            // Uses a modified formula adapted from the c_c2sc2_c function
            // from http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c
            t = 0; u = 0; v = 0;

            float xi = goal.X + (float)Math.Sin(goal.Orientation);
            float eta = goal.Y - 1 - (float)Math.Cos(goal.Orientation);

            float u1squared = xi * xi + eta * eta;
            if (u1squared < 16)
                return float.PositiveInfinity;

            float phi = (float)Math.Atan2(eta, xi);

            u = (float)Math.Sqrt(u1squared - 4) - 4;
            if (u < 0)
                return float.PositiveInfinity;

            float alpha = (float)Math.Atan2(2, u + 4);
            t = mod2PI(MathHelper.PiOver2 + phi + alpha);
            v = mod2PI(t - goal.Orientation);

            return t + u + v + MathHelper.Pi;
        }

        private static ReedsSheppActionSet LfRbpi2SbLbpi2Rfpath(float t, float u, float v)
        {
            ReedsSheppActionSet actions = new ReedsSheppActionSet();
            actions.AddAction(Steer.Left, Gear.Forward, t);
            actions.AddAction(Steer.Right, Gear.Backward, MathHelper.PiOver2);
            actions.AddAction(Steer.Straight, Gear.Backward, u);
            actions.AddAction(Steer.Left, Gear.Backward, MathHelper.PiOver2);
            actions.AddAction(Steer.Right, Gear.Forward, v);
            return actions;
        }

        private static bool isInvalidAngle(float theta)
        {
            return theta < 0 || theta > Math.PI;
        }

        private static float mod2PI(float theta)
        {
            while (theta < 0) theta += MathHelper.TwoPi;
            while (theta >= MathHelper.TwoPi) theta -= MathHelper.TwoPi;
            return theta;
        }
    }
}
