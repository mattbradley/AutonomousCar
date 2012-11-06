using Microsoft.Xna.Framework;
using Microsoft.Xna.Framework.Input;

namespace AutonomousCar.Helpers
{
    /// <summary>
    /// The Camera class implements a basic 3D camera. It has a 3D position, orientation, field of view, and aspect ratio.
    /// </summary>
    public class Camera
    {
        private float fieldOfView, aspectRatio, nearPlaneDistance, farPlaneDistance;
        private Vector3 position;
        private Matrix orientation, view, projection, viewProjection;

        private const float rotationSpeed = 0.025f;
        private const float moveSpeed = 0.5f;
        private const float zoomSpeed = 0.01f;
        private const float fovMin = float.Epsilon;
        private const float fovMax = MathHelper.Pi - 0.000001f;

        public float FieldOfView
        {
            get { return fieldOfView; }
            set
            {
                fieldOfView = MathHelper.Clamp(value, fovMin, fovMax);
                updateProjectionMatrix();
                updateViewProjectionMatrix();
            }
        }

        public float AspectRatio
        {
            get { return aspectRatio; }
            set
            {
                aspectRatio = value;
                updateProjectionMatrix();
                updateViewProjectionMatrix();
            }
        }

        public float NearPlaneDistance
        {
            get { return nearPlaneDistance; }
            set
            {
                nearPlaneDistance = value;
                updateProjectionMatrix();
                updateViewProjectionMatrix();
            }
        }

        public float FarPlaneDistance
        {
            get { return farPlaneDistance; }
            set
            {
                farPlaneDistance = value;
                updateProjectionMatrix();
                updateViewProjectionMatrix();
            }
        }

        public Vector3 Position
        {
            get { return position; }
            set
            {
                position = value;
                updateViewMatrix();
                updateViewProjectionMatrix();
            }
        }

        public Matrix View
        {
            get { return view; }
            set
            {
                view = value;
                updateViewProjectionMatrix();
            }
        }

        public Matrix Projection
        {
            get { return projection; }
            set
            {
                projection = value;
                updateViewProjectionMatrix();
            }
        }

        public Matrix ViewProjection
        {
            get { return viewProjection; }
        }

        public Vector3 Forward
        {
            get { return orientation.Forward; }
        }

        public Vector3 Backward
        {
            get { return orientation.Backward; }
        }

        public Vector3 Left
        {
            get { return orientation.Left; }
        }

        public Vector3 Right
        {
            get { return orientation.Right; }
        }

        public Vector3 Up
        {
            get { return orientation.Up; }
        }

        public Vector3 Down
        {
            get { return orientation.Down; }
        }

        public Camera(float fov, float viewportAspectRatio, float nearPlane, float farPlane)
        {
            position = Vector3.Zero;
            orientation = Matrix.Identity;

            fieldOfView = MathHelper.Clamp(fov, fovMin, fovMax);
            aspectRatio = viewportAspectRatio;
            nearPlaneDistance = nearPlane;
            farPlaneDistance = farPlane;

            updateViewMatrix();
            updateProjectionMatrix();
            updateViewProjectionMatrix();
        }

        private void updateViewMatrix()
        {
            view = Matrix.CreateLookAt(position, position + Forward, Up);
        }

        private void updateProjectionMatrix()
        {
            projection = Matrix.CreatePerspectiveFieldOfView(fieldOfView, aspectRatio, nearPlaneDistance, farPlaneDistance);
        }

        private void updateViewProjectionMatrix()
        {
            viewProjection = view * projection;
        }

        public void HandleInput()
        {
            KeyboardState kbState = Keyboard.GetState();
            GamePadState gpState = GamePad.GetState(PlayerIndex.One);

            Matrix rotation = Matrix.Identity;

            if (kbState.IsKeyDown(Keys.NumPad4))
                rotation *= Matrix.CreateFromAxisAngle(Up, rotationSpeed);
            if (kbState.IsKeyDown(Keys.NumPad6))
                rotation *= Matrix.CreateFromAxisAngle(Up, -rotationSpeed);
            if (kbState.IsKeyDown(Keys.NumPad8))
                rotation *= Matrix.CreateFromAxisAngle(Right, -rotationSpeed);
            if (kbState.IsKeyDown(Keys.NumPad2))
                rotation *= Matrix.CreateFromAxisAngle(Right, rotationSpeed);
            if (kbState.IsKeyDown(Keys.NumPad7))
                rotation *= Matrix.CreateFromAxisAngle(Forward, -rotationSpeed);
            if (kbState.IsKeyDown(Keys.NumPad9))
                rotation *= Matrix.CreateFromAxisAngle(Forward, rotationSpeed);

            rotation *= Matrix.CreateFromAxisAngle(Up, -rotationSpeed * gpState.ThumbSticks.Right.X);
            rotation *= Matrix.CreateFromAxisAngle(Right, -rotationSpeed * gpState.ThumbSticks.Right.Y);
            rotation *= Matrix.CreateFromAxisAngle(Forward, rotationSpeed * (-gpState.Triggers.Left + gpState.Triggers.Right));

            if (rotation != Matrix.Identity)
                orientation *= rotation;

            Vector3 translation = Vector3.Zero;

            if (kbState.IsKeyDown(Keys.Up))
                if (kbState.IsKeyDown(Keys.LeftShift) || kbState.IsKeyDown(Keys.RightShift))
                    translation += Forward * moveSpeed;
                else
                    translation += Up * moveSpeed;
            if (kbState.IsKeyDown(Keys.Down))
                if (kbState.IsKeyDown(Keys.LeftShift) || kbState.IsKeyDown(Keys.RightShift))
                    translation += Backward * moveSpeed;
                else
                    translation += Down * moveSpeed;
            if (kbState.IsKeyDown(Keys.Left))
                translation += Left * moveSpeed;
            if (kbState.IsKeyDown(Keys.Right))
                translation += Right * moveSpeed;

            translation += Forward * moveSpeed * gpState.ThumbSticks.Left.Y;
            translation += Right * moveSpeed * gpState.ThumbSticks.Left.X;
            if (gpState.IsButtonDown(Buttons.LeftShoulder))
                translation += Down * moveSpeed;
            if (gpState.IsButtonDown(Buttons.RightShoulder))
                translation += Up * moveSpeed;

            if (translation != Vector3.Zero)
                position += translation;

            if ((rotation != Matrix.Identity) || (translation != Vector3.Zero))
                updateViewMatrix();

            float zoom = 0;

            if (kbState.IsKeyDown(Keys.Add))
                zoom -= zoomSpeed;
            if (kbState.IsKeyDown(Keys.Subtract))
                zoom += zoomSpeed;

            if (zoom != 0)
            {
                fieldOfView += zoom;
                fieldOfView = MathHelper.Clamp(fieldOfView, fovMin, fovMax);
                updateProjectionMatrix();
            }

            if ((rotation != Matrix.Identity) || (translation != Vector3.Zero) || (zoom != 0))
                updateViewProjectionMatrix();
        }
    }
}
