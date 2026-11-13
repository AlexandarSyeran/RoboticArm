using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media.Media3D;

namespace RoboticArm.Views
{
    public sealed class CameraController
    {
        private Viewport3D? _viewport;
        private PerspectiveCamera? _camera;

        private double _theta = -Math.PI / 2.0;
        private double _phi = Math.PI * 0.08;
        private double _radius = 9.0;
        private Point3D _target = new(0, 0.30, 0);

        private Point _lastMouse;
        private bool _isLeftDown;
        private bool _isRightDown;

        public void Attach(Viewport3D viewport)
        {
            _viewport = viewport;
            _camera = new PerspectiveCamera { FieldOfView = 45 };
            viewport.Camera = _camera;

            viewport.MouseDown += OnMouseDown;
            viewport.MouseUp += OnMouseUp;
            viewport.MouseMove += OnMouseMove;
            viewport.MouseWheel += OnMouseWheel;

            UpdateCamera();
        }

        private void OnMouseDown(object sender, MouseButtonEventArgs e)
        {
            _lastMouse = e.GetPosition(_viewport);
            if (e.ChangedButton == MouseButton.Left)
                _isLeftDown = true;
            if (e.ChangedButton == MouseButton.Right)
                _isRightDown = true;

            Mouse.Capture(_viewport);
        }

        private void OnMouseUp(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left)
                _isLeftDown = false;
            if (e.ChangedButton == MouseButton.Right)
                _isRightDown = false;

            if (!_isLeftDown && !_isRightDown)
                Mouse.Capture(null);
        }

        private void OnMouseMove(object sender, MouseEventArgs e)
        {
            if (_viewport == null)
                return;

            var position = e.GetPosition(_viewport);
            double dx = position.X - _lastMouse.X;
            double dy = position.Y - _lastMouse.Y;
            _lastMouse = position;

            if (_isLeftDown)
            {
                _theta += dx * 0.01;
                _phi = Math.Clamp(_phi - dy * 0.01, -Math.PI / 2 + 0.01, Math.PI / 2 - 0.01);
                UpdateCamera();
            }
            else if (_isRightDown)
            {
                _radius = Math.Max(1.0, _radius + dy * 0.02);
                UpdateCamera();
            }
        }

        private void OnMouseWheel(object sender, MouseWheelEventArgs e)
        {
            _radius = Math.Max(1.0, _radius * (e.Delta > 0 ? 0.9 : 1.1));
            UpdateCamera();
        }

        private void UpdateCamera()
        {
            if (_camera == null)
                return;

            double x = _radius * Math.Cos(_theta) * Math.Cos(_phi);
            double z = _radius * Math.Sin(_theta) * Math.Cos(_phi);
            double y = _radius * Math.Sin(_phi);

            var position = new Point3D(
                _target.X + x,
                _target.Y + y,
                _target.Z + z);

            _camera.Position = position;
            _camera.LookDirection = new Vector3D(
                _target.X - position.X,
                _target.Y - position.Y,
                _target.Z - position.Z);
            _camera.UpDirection = new Vector3D(0, 1, 0);
        }
    }
}
