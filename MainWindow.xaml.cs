using System.Windows;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using RoboticArm.Rendering;
using RoboticArm.Services;
using RoboticArm.ViewModels;
using RoboticArm.Views;

namespace RoboticArm
{
    public partial class MainWindow : Window
    {
        private readonly MainViewModel _viewModel;
        private readonly RobotSceneRenderer _renderer;
        private readonly CameraController _camera = new();
        private bool _isDraggingTarget;

        public MainWindow()
        {
            InitializeComponent();

            _viewModel = new MainViewModel(new WpfDialogService(this));
            DataContext = _viewModel;

            _renderer = new RobotSceneRenderer(MainViewport, _viewModel);
            _renderer.Build();

            _camera.Attach(MainViewport);

            Closed += (_, _) =>
            {
                _renderer.Dispose();
                _viewModel.Dispose();
            };
        }

        private void Viewport3D_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            bool ctrl = Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl);
            if (ctrl)
            {
                _isDraggingTarget = true;
                MainViewport.CaptureMouse();
            }

            HitTestIkTarget(e.GetPosition(MainViewport));
        }

        private void Viewport3D_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isDraggingTarget && e.LeftButton == MouseButtonState.Pressed)
                HitTestIkTarget(e.GetPosition(MainViewport));
        }

        private void Viewport3D_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (!_isDraggingTarget)
                return;

            _isDraggingTarget = false;
            MainViewport.ReleaseMouseCapture();
        }

        private void HitTestIkTarget(Point position)
        {
            VisualTreeHelper.HitTest(
                MainViewport,
                null,
                result =>
                {
                    if (result is RayHitTestResult ray)
                    {
                        _viewModel.SetIkTarget(ray.PointHit, _isDraggingTarget);
                        return HitTestResultBehavior.Stop;
                    }

                    return HitTestResultBehavior.Continue;
                },
                new PointHitTestParameters(position));
        }
    }
}
