using System;
using System.Collections;
using C5;

namespace AutonomousCar.PathFinding.ReedsShepp
{
    /// <summary>
    /// The PathWords enum lists every possible Reeds-Shepp pattern. L, S, or R described the steering direction (left, straight, or right),
    /// and f or b describe the gear (forward or backward).
    /// </summary>
    public enum PathWords
    {
        LfSfLf, // Reeds-Shepp 8.1: CSC, same turn
        LbSbLb,
        RfSfRf,
        RbSbRb,

        LfSfRf, // Reeds-Shepp 8.2: CSC, different turn
        LbSbRb,
        RfSfLf,
        RbSbLb,

        LfRbLf, // Reeds-Shepp 8.3: C|C|C
        LbRfLb,
        RfLbRf,
        RbLfRb,

        LfRbLb, // Reeds-Shepp 8.4: C|CC
        LbRfLf,
        RfLbRb,
        RbLfRf,

        LfRfLb, // Reeds-Shepp 8.4: CC|C
        LbRbLf,
        RfLfRb,
        RbLbRf,

        LfRufLubRb, // Reeds-Shepp 8.7: CCu|CuC 
        LbRubLufRf,
        RfLufRubLb,
        RbLubRufLf,

        LfRubLubRf, // Reeds-Shepp 8.8: C|CuCu|C
        LbRufLufRb,
        RfLubRubLf,
        RbLufRufLb,

        LfRbpi2SbLb, // Reeds-Shepp 8.9: C|C(pi/2)SC, same turn
        LbRfpi2SfLf,
        RfLbpi2SbRb,
        RbLfpi2SfRf,

        LfRbpi2SbRb, // Reeds-Shepp 8.10: C|C(pi/2)SC, different turn
        LbRfpi2SfRf,
        RfLbpi2SbLb,
        RbLfpi2SfLf,

        LfSfRfpi2Lb, // Reeds-Shepp 8.9 (reversed): CSC(pi/2)|C, same turn
        LbSbRbpi2Lf,
        RfSfLfpi2Rb,
        RbSbLbpi2Rf,

        LfSfLfpi2Rb, // Reeds-Shepp 8.10 (reversed): CSC(pi/2)|C, different turn
        LbSbLbpi2Rf,
        RfSfRfpi2Lb,
        RbSbRbpi2Lf,

        LfRbpi2SbLbpi2Rf, // Reeds-Shepp 8.11: C|C(pi/2)SC(pi/2)|C
        LbRfpi2SfLfpi2Rb,
        RfLbpi2SbRbpi2Lf,
        RbLfpi2SfRfpi2Lb
    }

    /// <summary>
    /// The ReedsSheppAction class represents a single steering and motion action over some length.
    /// </summary>
    public class ReedsSheppAction
    {
        public Steer Steer;
        public Gear Gear;
        public float Length;

        public ReedsSheppAction(Steer steer, Gear gear, float length)
        {
            Steer = steer;
            Gear = gear;
            Length = length;
        }
    }

    /// <summary>
    /// The ReedsSheppActionSet class is a set of ReedsSheppActions. As actions are added, their lengths are summed together.
    /// The total cost of the set can be calculated using a reverse gear cost and a gear switch cost.
    /// </summary>
    public class ReedsSheppActionSet
    {
        public ArrayList<ReedsSheppAction> Actions { get; set; }
        public float Length { get; set; }

        public ReedsSheppActionSet() : this(0f) { }
        public ReedsSheppActionSet(float length)
        {
            Actions = new ArrayList<ReedsSheppAction>();
            Length = length;
        }

        public void AddAction(Steer steer, Gear gear, float length)
        {
            Actions.Add(new ReedsSheppAction(steer, gear, length));
            Length += Math.Abs(length);
        }

        public float CalculateCost(float unit, float reverseCostMultiplier, float gearSwitchCost)
        {
            if (reverseCostMultiplier == 1f && gearSwitchCost == 0f)
                return Length * unit;

            if (Length == float.PositiveInfinity || Actions.IsEmpty)
                return float.PositiveInfinity;

            float cost = 0;

            Gear prevGear = Actions[0].Gear;
            foreach (ReedsSheppAction a in Actions)
            {
                float actionCost = a.Length * unit;
                if (a.Gear == Gear.Backward)
                    actionCost *= reverseCostMultiplier;
                if (a.Gear != prevGear)
                    actionCost += gearSwitchCost;

                prevGear = a.Gear;
                cost += actionCost;
            }

            return cost;
        }
    }
}
