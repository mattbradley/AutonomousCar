using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using FarseerPhysics.Common;
using FarseerPhysics.Dynamics;
using Microsoft.Xna.Framework;

namespace FarseerPhysics.Controllers
{
    public class TensorDampingController : Controller
    {

        /// <summary>
        /// Tensor to use in damping model
        /// Some examples (matrixes in format (row1; row2) )
        ///(-a 0;0 -a)          Standard isotropic damping with strength a
        ///(0 a;-a 0)           Electron in fixed field - a force at right angles to velocity with proportional magnitude
        ///(-a 0;0 -b)          Differing x and y damping. Useful e.g. for top-down wheels.
        ///By the way, tensor in this case just means matrix, don't let the terminology get you down.
        /// </summary>
        Mat22 T;

        /// <summary>
        /// Set this to a positive number to clamp the maximum amount of damping done.
        /// Typically one wants maxTimestep to be 1/(max eigenvalue of T), so that damping will never cause something to reverse direction
        /// </summary>
        float MaxTimestep;

        private List<Body> _bodies = new List<Body>();

        public TensorDampingController() : base(ControllerType.TensorDampingController)
        {

        }

        /// Sets damping independantly along the x and y axes
        public void SetAxisAligned(float xDamping, float yDamping)
        {
            T.Col1.X = -xDamping;
            T.Col1.Y = 0;
            T.Col2.X = 0;
            T.Col2.Y = -yDamping;
            if (xDamping > 0 || yDamping > 0)
            {
                MaxTimestep = 1 / Math.Max(xDamping, yDamping);
            }
            else
            {
                MaxTimestep = 0;
            }
        }

        public override void Update(float dt)
        {
            if (dt > MaxTimestep && MaxTimestep > 0)
                dt = MaxTimestep;
            foreach (Body body in _bodies)
            {
                if (!body.Awake)
                    continue;

                Vector2 damping = body.GetWorldVector(MathUtils.Multiply(ref T, body.GetLocalVector(body.LinearVelocity)));
                body.LinearVelocity += dt * damping;
            }
        }

        public void AddBody(Body body)
        {
            _bodies.Add(body);
        }

        public void RemoveBody(Body body)
        {
            _bodies.Remove(body);
        }
    }
}
