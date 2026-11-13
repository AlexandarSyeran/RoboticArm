using System.IO;
using System.Text.Json;
using System.Windows.Input;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using RoboticArm.Models;
using RoboticArm.Mvvm;
using RoboticArm.Services;

namespace RoboticArm.ViewModels
{
    public sealed class MainViewModel : ObservableObject, IDisposable
    {
        private const double TimerMilliseconds = 30.0;

        private readonly IDialogService _dialogs;
        private readonly AutomationController _automation;
        private readonly KeyframePlayer _keyframes = new();
        private readonly List<AutomationItem> _items = new();
        private readonly List<Obstacle> _obstacles = new();

        private readonly DispatcherTimer _automationTimer;
        private readonly DispatcherTimer _waveTimer;
        private readonly DispatcherTimer _wristTimer;
        private readonly DispatcherTimer _trajectoryTimer;

        private double _waveTime;
        private int _obstacleTestNumber;
        private bool _obstacleTestActive;
        private double _obstacleTestKeyframeTime;

        private double _automationSpeed = 0.55;
        private string _arrivalDelayText = "2";
        private string _actionDelayText = "0.7";
        private string _ikX = "2.5";
        private string _ikY = "0.5";
        private string _ikZ = "0.0";
        private string _ikRx = "0";
        private string _ikRy = "0";
        private string _ikRz = "0";
        private bool _useOrientation = true;
        private bool _showWristAxes = true;
        private int _toolModeIndex;
        private string _objectStatus = "Object: free";
        private string _automationStatus = "Automation: idle";
        private string _automationDiagnostic = string.Empty;
        private bool _hasDiagnostic;

        private bool _demoAttached;
        private Vector3D _demoToolOffset;
        private Point3D _demoObjectPosition;

        public MainViewModel(IDialogService dialogs)
        {
            _dialogs = dialogs;
            Robot = new RobotArmModel();

            var folded = Robot.ForwardKinematics();
            HomePosition = folded.EePos;
            PickupZone = new Point3D(folded.EePos.X, 0.30, 0);
            DropTarget = new Point3D(-PickupZone.X, PickupZone.Y, PickupZone.Z);
            _demoObjectPosition = new Point3D(
                folded.EePos.X,
                Math.Max(folded.EePos.Y, 0.30),
                folded.EePos.Z);

            _automation = new AutomationController(
                Robot, _items, _obstacles, PickupZone, DropTarget, HomePosition);

            _automation.StatusChanged += OnAutomationStatusChanged;
            _automation.DiagnosticChanged += () => SetAutomationDiagnostic(_automation.Diagnostic);
            _automation.Finished += OnAutomationFinished;

            _automationTimer = CreateTimer(AutomationTick);
            _waveTimer = CreateTimer(WaveTick);
            _wristTimer = CreateTimer(WristRotateTick);
            _trajectoryTimer = CreateTimer(TrajectoryTick);

            ResetCommand = new RelayCommand(ResetJoints);
            Solve6DCommand = new RelayCommand(Solve6D);
            RecordKeyframeCommand = new RelayCommand(RecordKeyframe);
            ClearKeyframesCommand = new RelayCommand(ClearKeyframes);
            ExportKeyframesCommand = new RelayCommand(ExportKeyframes);
            ImportKeyframesCommand = new RelayCommand(ImportKeyframes);
            PlayTrajectoryCommand = new RelayCommand(PlayTrajectory);
            StopTrajectoryCommand = new RelayCommand(StopTrajectory);
            OpenGripperCommand = new RelayCommand(() => GripperOpen = 1.0);
            CloseGripperCommand = new RelayCommand(CloseGripper);
            StartWristRotateCommand = new RelayCommand(() => _wristTimer.Start());
            StopWristRotateCommand = new RelayCommand(() => _wristTimer.Stop());
            ActivateToolCommand = new RelayCommand(() => ToolActivated?.Invoke((ToolMode)ToolModeIndex));
            PlaceObjectCommand = new RelayCommand(PlaceObject);
            PickObjectCommand = new RelayCommand(TryPickObject);
            ReleaseObjectCommand = new RelayCommand(ReleaseObject);
            StartAutomationCommand = new RelayCommand(StartAutomation);
            StopAutomationCommand = new RelayCommand(StopAutomation);
            ResetAutomationCommand = new RelayCommand(ResetAutomationObjects);
            TestObstacleAvoidanceCommand = new RelayCommand(TestObstacleAvoidance);
            PlayWaveCommand = new RelayCommand(PlayWave);
            StopWaveCommand = new RelayCommand(() => _waveTimer.Stop());

            ResetAutomationObjects();
        }

        public RobotArmModel Robot { get; }
        public Point3D PickupZone { get; }
        public Point3D DropTarget { get; }
        public Point3D HomePosition { get; }
        public IReadOnlyList<AutomationItem> Items => _items;
        public IReadOnlyList<Obstacle> Obstacles => _obstacles;

        public bool DemoAttached => _demoAttached;
        public Point3D DemoObjectPosition => _demoObjectPosition;
        public ToolMode ToolMode => (ToolMode)ToolModeIndex;
        public bool IsAutomationRunning => _automation.IsRunning;

        public event Action? PoseChanged;
        public event Action? AutomationSceneChanged;
        public event Action<ToolMode>? ToolActivated;
        public event Action<bool>? WristAxesVisibilityChanged;

        public double Joint1
        {
            get => Robot.Angles[0];
            set => SetJoint(0, value);
        }

        public double Joint2
        {
            get => Robot.Angles[1];
            set => SetJoint(1, value);
        }

        public double Joint3
        {
            get => Robot.Angles[2];
            set => SetJoint(2, value);
        }

        public double Joint4
        {
            get => Robot.Angles[3];
            set => SetJoint(3, value);
        }

        public string Joint1Text => FormatAngle(Joint1);
        public string Joint2Text => FormatAngle(Joint2);
        public string Joint3Text => FormatAngle(Joint3);
        public string Joint4Text => FormatAngle(Joint4);

        public double WristRoll
        {
            get => Robot.WristRoll;
            set
            {
                if (Math.Abs(Robot.WristRoll - value) < 1e-9)
                    return;

                Robot.WristRoll = value;
                OnPropertyChanged();
                OnPropertyChanged(nameof(WristRollText));
                OnPoseChanged();
            }
        }

        public string WristRollText => FormatAngle(WristRoll);

        public double GripperOpen
        {
            get => Robot.GripperOpen;
            set
            {
                if (Math.Abs(Robot.GripperOpen - value) < 1e-9)
                    return;

                Robot.GripperOpen = value;
                OnPropertyChanged();
                OnPoseChanged();
            }
        }

        public double AutomationSpeed
        {
            get => _automationSpeed;
            set
            {
                if (!SetProperty(ref _automationSpeed, value))
                    return;

                _automation.SpeedMultiplier = value;
                OnPropertyChanged(nameof(AutomationSpeedText));
            }
        }

        public string AutomationSpeedText => $"{AutomationSpeed:P0}";

        public string ArrivalDelayText
        {
            get => _arrivalDelayText;
            set => SetProperty(ref _arrivalDelayText, value);
        }

        public string ActionDelayText
        {
            get => _actionDelayText;
            set => SetProperty(ref _actionDelayText, value);
        }

        public string IkX
        {
            get => _ikX;
            set => SetProperty(ref _ikX, value);
        }

        public string IkY
        {
            get => _ikY;
            set => SetProperty(ref _ikY, value);
        }

        public string IkZ
        {
            get => _ikZ;
            set => SetProperty(ref _ikZ, value);
        }

        public string IkRx
        {
            get => _ikRx;
            set => SetProperty(ref _ikRx, value);
        }

        public string IkRy
        {
            get => _ikRy;
            set => SetProperty(ref _ikRy, value);
        }

        public string IkRz
        {
            get => _ikRz;
            set => SetProperty(ref _ikRz, value);
        }

        public bool UseOrientation
        {
            get => _useOrientation;
            set => SetProperty(ref _useOrientation, value);
        }

        public bool ShowWristAxes
        {
            get => _showWristAxes;
            set
            {
                if (SetProperty(ref _showWristAxes, value))
                    WristAxesVisibilityChanged?.Invoke(value);
            }
        }

        public int ToolModeIndex
        {
            get => _toolModeIndex;
            set => SetProperty(ref _toolModeIndex, value);
        }

        public string ObjectStatus
        {
            get => _objectStatus;
            private set => SetProperty(ref _objectStatus, value);
        }

        public string AutomationStatus
        {
            get => _automationStatus;
            private set => SetProperty(ref _automationStatus, value);
        }

        public string AutomationDiagnostic
        {
            get => _automationDiagnostic;
            private set => SetProperty(ref _automationDiagnostic, value);
        }

        public bool HasDiagnostic
        {
            get => _hasDiagnostic;
            private set => SetProperty(ref _hasDiagnostic, value);
        }

        public ICommand ResetCommand { get; }
        public ICommand Solve6DCommand { get; }
        public ICommand RecordKeyframeCommand { get; }
        public ICommand ClearKeyframesCommand { get; }
        public ICommand ExportKeyframesCommand { get; }
        public ICommand ImportKeyframesCommand { get; }
        public ICommand PlayTrajectoryCommand { get; }
        public ICommand StopTrajectoryCommand { get; }
        public ICommand OpenGripperCommand { get; }
        public ICommand CloseGripperCommand { get; }
        public ICommand StartWristRotateCommand { get; }
        public ICommand StopWristRotateCommand { get; }
        public ICommand ActivateToolCommand { get; }
        public ICommand PlaceObjectCommand { get; }
        public ICommand PickObjectCommand { get; }
        public ICommand ReleaseObjectCommand { get; }
        public ICommand StartAutomationCommand { get; }
        public ICommand StopAutomationCommand { get; }
        public ICommand ResetAutomationCommand { get; }
        public ICommand TestObstacleAvoidanceCommand { get; }
        public ICommand PlayWaveCommand { get; }
        public ICommand StopWaveCommand { get; }

        public void SetIkTarget(Point3D point, bool solveLive)
        {
            IkX = point.X.ToString("F2");
            IkY = point.Y.ToString("F2");
            IkZ = point.Z.ToString("F2");

            if (solveLive)
                Solve6DLive(point);
        }

        public void Dispose()
        {
            _automationTimer.Stop();
            _waveTimer.Stop();
            _wristTimer.Stop();
            _trajectoryTimer.Stop();
        }

        private static DispatcherTimer CreateTimer(Action tick)
        {
            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(TimerMilliseconds) };
            timer.Tick += (_, _) => tick();
            return timer;
        }

        private static string FormatAngle(double value) => $"{Math.Round(value, 1)}°";

        private void SetJoint(int index, double value)
        {
            if (Math.Abs(Robot.Angles[index] - value) < 1e-9)
                return;

            Robot.Angles[index] = value;
            RaiseJointChanged(index);
            OnPoseChanged();
        }

        private void RaiseJointChanged(int index)
        {
            switch (index)
            {
                case 0:
                    OnPropertyChanged(nameof(Joint1));
                    OnPropertyChanged(nameof(Joint1Text));
                    break;
                case 1:
                    OnPropertyChanged(nameof(Joint2));
                    OnPropertyChanged(nameof(Joint2Text));
                    break;
                case 2:
                    OnPropertyChanged(nameof(Joint3));
                    OnPropertyChanged(nameof(Joint3Text));
                    break;
                case 3:
                    OnPropertyChanged(nameof(Joint4));
                    OnPropertyChanged(nameof(Joint4Text));
                    break;
            }
        }

        private void SyncJointsFromModel()
        {
            OnPropertyChanged(nameof(Joint1));
            OnPropertyChanged(nameof(Joint2));
            OnPropertyChanged(nameof(Joint3));
            OnPropertyChanged(nameof(Joint4));
            OnPropertyChanged(nameof(Joint1Text));
            OnPropertyChanged(nameof(Joint2Text));
            OnPropertyChanged(nameof(Joint3Text));
            OnPropertyChanged(nameof(Joint4Text));
            OnPropertyChanged(nameof(GripperOpen));
            OnPoseChanged();
        }

        private void OnPoseChanged()
        {
            UpdateAttachedObjects();
            PoseChanged?.Invoke();
        }

        private void UpdateAttachedObjects()
        {
            var fk = Robot.ForwardKinematics();
            var toolOrientation = Robot.ToolOrientation(fk);

            if (_demoAttached)
            {
                var offset = toolOrientation.Transform(_demoToolOffset);
                _demoObjectPosition = new Point3D(
                    fk.EePos.X + offset.X,
                    fk.EePos.Y + offset.Y,
                    fk.EePos.Z + offset.Z);
            }

            _automation.UpdateAttachedPose();
        }

        private void ResetJoints()
        {
            SetJoint(0, 0);
            SetJoint(1, 0);
            SetJoint(2, 0);
            SetJoint(3, 0);
            WristRoll = 0;
        }

        private void Solve6D()
        {
            if (!TryReadTarget(out var position, out var rotation))
                return;

            Robot.Solve6D(position, rotation, UseOrientation);
            SyncJointsFromModel();
        }

        private void Solve6DLive(Point3D point)
        {
            double.TryParse(IkRx, out double rx);
            double.TryParse(IkRy, out double ry);
            double.TryParse(IkRz, out double rz);

            var rotation = RobotArmModel.EulerToRotation(rx, ry, rz);
            Robot.Solve6D(point, rotation, UseOrientation, 10);
            SyncJointsFromModel();
        }

        private bool TryReadTarget(out Point3D position, out Matrix3D rotation)
        {
            position = default;
            rotation = Matrix3D.Identity;

            if (!double.TryParse(IkX, out double x))
            {
                _dialogs.ShowMessage("Invalid X");
                return false;
            }

            if (!double.TryParse(IkY, out double y))
            {
                _dialogs.ShowMessage("Invalid Y");
                return false;
            }

            if (!double.TryParse(IkZ, out double z))
            {
                _dialogs.ShowMessage("Invalid Z");
                return false;
            }

            if (!double.TryParse(IkRx, out double rx))
            {
                _dialogs.ShowMessage("Invalid Rx");
                return false;
            }

            if (!double.TryParse(IkRy, out double ry))
            {
                _dialogs.ShowMessage("Invalid Ry");
                return false;
            }

            if (!double.TryParse(IkRz, out double rz))
            {
                _dialogs.ShowMessage("Invalid Rz");
                return false;
            }

            position = new Point3D(x, y, z);
            rotation = RobotArmModel.EulerToRotation(rx, ry, rz);
            return true;
        }

        private void RecordKeyframe()
        {
            _keyframes.Record(Robot.Angles);
            var last = _keyframes.Keyframes[^1];
            _dialogs.ShowMessage($"Recorded keyframe #{_keyframes.Keyframes.Count} at t={last.Time:F2}s");
        }

        private void ClearKeyframes()
        {
            _keyframes.Keyframes.Clear();
            _dialogs.ShowMessage("Keyframes cleared");
        }

        private void PlayTrajectory()
        {
            if (!_keyframes.Play())
            {
                _dialogs.ShowMessage("Need at least 2 keyframes to play.");
                return;
            }

            _trajectoryTimer.Start();
        }

        private void StopTrajectory()
        {
            _keyframes.Stop();
            _trajectoryTimer.Stop();
        }

        private void ExportKeyframes()
        {
            if (_keyframes.Keyframes.Count == 0)
            {
                _dialogs.ShowMessage("No keyframes to export.");
                return;
            }

            var path = _dialogs.ShowSaveFile("JSON files (*.json)|*.json|All files|*.*");
            if (path == null)
                return;

            try
            {
                var options = new JsonSerializerOptions { WriteIndented = true };
                File.WriteAllText(path, JsonSerializer.Serialize(_keyframes.Keyframes, options));
                _dialogs.ShowMessage("Exported keyframes.");
            }
            catch (Exception ex)
            {
                _dialogs.ShowMessage("Export failed: " + ex.Message);
            }
        }

        private void ImportKeyframes()
        {
            var path = _dialogs.ShowOpenFile("JSON files (*.json)|*.json|All files|*.*");
            if (path == null)
                return;

            try
            {
                var json = File.ReadAllText(path);
                var imported = JsonSerializer.Deserialize<List<Keyframe>>(json);
                if (imported != null)
                {
                    _keyframes.Replace(imported);
                    _dialogs.ShowMessage($"Imported {_keyframes.Keyframes.Count} keyframes.");
                }
            }
            catch (Exception ex)
            {
                _dialogs.ShowMessage("Import failed: " + ex.Message);
            }
        }

        private void CloseGripper()
        {
            GripperOpen = 0.0;
            TryPickObject();
        }

        private void PlaceObject()
        {
            _demoAttached = false;
            _demoToolOffset = default;

            var fk = Robot.ForwardKinematics();
            _demoObjectPosition = new Point3D(
                fk.EePos.X,
                Math.Max(fk.EePos.Y, 0.30),
                fk.EePos.Z);

            ObjectStatus = "Object: free";
            OnPoseChanged();
        }

        private void TryPickObject()
        {
            if (_demoAttached || _automation.AttachedItem != null)
                return;

            if (Robot.GripperOpen > 0.15)
            {
                ObjectStatus = "Object: close gripper first";
                return;
            }

            var fk = Robot.ForwardKinematics();
            var toolOrientation = Robot.ToolOrientation(fk);

            AutomationItem? closestItem = null;
            double closestDistance = double.MaxValue;

            foreach (var item in _items)
            {
                if (item.Completed || item.Attached)
                    continue;

                double candidateDistance = (item.Position - fk.EePos).Length;
                if (candidateDistance < closestDistance)
                {
                    closestDistance = candidateDistance;
                    closestItem = item;
                }
            }

            var objectPosition = closestItem?.Position ?? _demoObjectPosition;
            var distance = objectPosition - fk.EePos;
            if (distance.Length > RobotArmModel.PickDistance)
            {
                ObjectStatus = "Object: move it closer to gripper";
                return;
            }

            var inverse = toolOrientation;
            inverse.Invert();
            var toolOffset = inverse.Transform(distance);

            if (closestItem != null)
            {
                _automation.AttachManually(closestItem, toolOffset);
            }
            else
            {
                _demoAttached = true;
                _demoToolOffset = toolOffset;
            }

            ObjectStatus = "Object: attached";
            OnPoseChanged();
        }

        private void ReleaseObject()
        {
            _demoAttached = false;
            _demoToolOffset = default;
            _automation.DetachAttached();
            ObjectStatus = "Object: released";
            OnPoseChanged();
        }

        private void StartAutomation()
        {
            SetAutomationDiagnostic(string.Empty);
            _automation.ArrivalDelay = ParseDelay(ArrivalDelayText, 2.0);
            _automation.ActionDelay = ParseDelay(ActionDelayText, 0.7);
            _automation.SpeedMultiplier = AutomationSpeed;

            if (_automation.Start())
                _automationTimer.Start();
        }

        private void StopAutomation()
        {
            _automationTimer.Stop();
            _automation.Stop("Automation stopped.");

            if (_obstacleTestActive)
            {
                _obstacleTestActive = false;
                _obstacles.Clear();
                AutomationSceneChanged?.Invoke();
            }
        }

        private void ResetAutomationObjects()
        {
            _automationTimer.Stop();
            _automation.Stop("Automation stopped.");

            _items.Clear();
            var positions = new[]
            {
                new Point3D(PickupZone.X - 0.45, 0.46, PickupZone.Z - 0.35),
                new Point3D(PickupZone.X + 0.45, 0.46, PickupZone.Z - 0.35),
                new Point3D(PickupZone.X - 0.45, 0.46, PickupZone.Z + 0.35),
                new Point3D(PickupZone.X + 0.45, 0.46, PickupZone.Z + 0.35)
            };

            for (int i = 0; i < positions.Length; i++)
            {
                _items.Add(new AutomationItem
                {
                    StartPosition = positions[i],
                    Position = positions[i],
                    PaletteIndex = i
                });
            }

            _automation.ResetState();
            Robot.ResetPose();

            _demoAttached = false;
            _demoToolOffset = default;

            SyncJointsFromModel();
            OnPropertyChanged(nameof(WristRoll));
            OnPropertyChanged(nameof(WristRollText));
            OnPropertyChanged(nameof(GripperOpen));
            AutomationSceneChanged?.Invoke();
        }

        private void TestObstacleAvoidance()
        {
            StopAutomation();
            ResetAutomationObjects();
            GenerateObstacleLayout();

            _keyframes.Keyframes.Clear();
            _obstacleTestKeyframeTime = 0;
            _obstacleTestActive = true;
            AutomationStatus = $"Obstacle test {_obstacleTestNumber}: starting";

            StartAutomation();
        }

        private void GenerateObstacleLayout()
        {
            _obstacles.Clear();
            var random = new Random(++_obstacleTestNumber * 7919);
            var forbidden = new[] { PickupZone, DropTarget };

            for (int i = 0; i < 6; i++)
            {
                Point3D center;
                var halfSize = new Vector3D(0.16, 0.16, 0.16);
                int attempts = 0;

                do
                {
                    double side = i % 2 == 0 ? -1.0 : 1.0;
                    center = new Point3D(
                        -2.6 + random.NextDouble() * 5.2,
                        PickupZone.Y + 0.5,
                        side * (1.45 + random.NextDouble() * 1.8));
                    attempts++;
                }
                while (attempts < 100 &&
                       (IsNearZone(center, halfSize, forbidden[0], 0.75) ||
                        IsNearZone(center, halfSize, forbidden[1], 0.75) ||
                        OverlapsExistingObstacle(center, halfSize)));

                _obstacles.Add(new Obstacle { Center = center, HalfSize = halfSize });
            }

            AutomationSceneChanged?.Invoke();
        }

        private static bool IsNearZone(Point3D center, Vector3D halfSize, Point3D zone, double margin)
        {
            return Math.Abs(center.X - zone.X) < halfSize.X + margin &&
                   Math.Abs(center.Z - zone.Z) < halfSize.Z + margin;
        }

        private bool OverlapsExistingObstacle(Point3D center, Vector3D halfSize)
        {
            foreach (var obstacle in _obstacles)
            {
                if (Math.Abs(center.X - obstacle.Center.X) < halfSize.X + obstacle.HalfSize.X + 0.15 &&
                    Math.Abs(center.Z - obstacle.Center.Z) < halfSize.Z + obstacle.HalfSize.Z + 0.15)
                {
                    return true;
                }
            }

            return false;
        }

        private void PlayWave()
        {
            _waveTime = 0;
            _waveTimer.Start();
        }

        private void AutomationTick()
        {
            if (_obstacleTestActive)
            {
                _obstacleTestKeyframeTime += TimerMilliseconds / 1000.0;
                _keyframes.Keyframes.Add(new Keyframe
                {
                    Time = _obstacleTestKeyframeTime,
                    Angles = (double[])Robot.Angles.Clone()
                });
            }

            _automation.Tick();
            SyncJointsFromModel();

            if (!_automation.IsRunning)
                _automationTimer.Stop();
        }

        private void WaveTick()
        {
            _waveTime += TimerMilliseconds / 1000.0;
            Robot.Angles[2] = 30 * Math.Sin(_waveTime * 2.0);
            Robot.Angles[3] = 20 * Math.Sin(_waveTime * 3.0 + 0.5);
            SyncJointsFromModel();
        }

        private void WristRotateTick()
        {
            double angle = Robot.WristRoll + 2.5;
            if (angle >= 360)
                angle -= 360;

            WristRoll = angle;
        }

        private void TrajectoryTick()
        {
            if (!_keyframes.Tick(TimerMilliseconds / 1000.0, Robot.Angles))
            {
                _trajectoryTimer.Stop();
                SyncJointsFromModel();
                return;
            }

            SyncJointsFromModel();
        }

        private void OnAutomationStatusChanged()
        {
            AutomationStatus = _automation.Status;
            OnPropertyChanged(nameof(IsAutomationRunning));
        }

        private void OnAutomationFinished()
        {
            _automationTimer.Stop();

            if (_obstacleTestActive)
            {
                if (_automation.Diagnostic != "Operation finished")
                {
                    _obstacles.Clear();
                    AutomationSceneChanged?.Invoke();
                }

                _obstacleTestActive = false;
            }

            OnPropertyChanged(nameof(IsAutomationRunning));
        }

        private void SetAutomationDiagnostic(string message)
        {
            AutomationDiagnostic = message ?? string.Empty;
            HasDiagnostic = !string.IsNullOrWhiteSpace(message);
        }

        private static double ParseDelay(string text, double fallback)
            => double.TryParse(text, out double value) && value >= 0 ? value : fallback;
    }
}
