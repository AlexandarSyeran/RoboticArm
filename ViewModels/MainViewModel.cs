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
        private readonly CuttingController _cutting;
        private readonly RadarScanner _radar;
        private readonly KeyframePlayer _keyframes = new();
        private readonly List<AutomationItem> _items = new();
        private readonly List<Obstacle> _obstacles = new();

        private readonly DispatcherTimer _automationTimer;
        private readonly DispatcherTimer _waveTimer;
        private readonly DispatcherTimer _wristTimer;
        private readonly DispatcherTimer _trajectoryTimer;
        private readonly DispatcherTimer _cutTimer;

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
        private string _cutStatus = "Cutting: idle";
        private string _toastMessage = string.Empty;
        private bool _hasToast;
        private bool _cutCompleted;
        private string _radarSummary = string.Empty;
        private string _radarReadout = string.Empty;

        public MainViewModel(IDialogService dialogs)
        {
            _dialogs = dialogs;
            Robot = new RobotArmModel();

            var folded = Robot.ForwardKinematics();
            HomePosition = folded.EePos;
            PickupZone = new Point3D(folded.EePos.X, 0.30, 0);
            DropTarget = new Point3D(-PickupZone.X, PickupZone.Y, PickupZone.Z);

            _automation = new AutomationController(
                Robot, _items, _obstacles, PickupZone, DropTarget, HomePosition);

            _automation.StatusChanged += OnAutomationStatusChanged;
            _automation.DiagnosticChanged += () => SetAutomationDiagnostic(_automation.Diagnostic);
            _automation.Finished += OnAutomationFinished;

            _cutting = new CuttingController(Robot, _items);
            _cutting.StatusChanged += OnCuttingStatusChanged;
            _cutting.Finished += OnCuttingFinished;
            _cutting.ItemCut += OnItemCut;

            _radar = new RadarScanner(_items, PickupZone);

            _automationTimer = CreateTimer(AutomationTick);
            _waveTimer = CreateTimer(WaveTick);
            _wristTimer = CreateTimer(WristRotateTick);
            _trajectoryTimer = CreateTimer(TrajectoryTick);
            _cutTimer = CreateTimer(CuttingTick);

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
            ActivateToolCommand = new RelayCommand(OnActivateTool);
            PlaceObjectCommand = new RelayCommand(StartManualPlace);
            PickObjectCommand = new RelayCommand(StartManualPick);
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

        public double Joint5
        {
            get => Robot.Angles[4];
            set => SetJoint(4, value);
        }

        public string Joint1Text => FormatAngle(Joint1);
        public string Joint2Text => FormatAngle(Joint2);
        public string Joint3Text => FormatAngle(Joint3);
        public string Joint4Text => FormatAngle(Joint4);
        public string Joint5Text => FormatAngle(Joint5);

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

        public string CutStatus
        {
            get => _cutStatus;
            private set => SetProperty(ref _cutStatus, value);
        }

        public string ToastMessage
        {
            get => _toastMessage;
            private set => SetProperty(ref _toastMessage, value);
        }

        public bool HasToast
        {
            get => _hasToast;
            private set => SetProperty(ref _hasToast, value);
        }

        public bool IsCutting => _cutting.IsRunning;
        public bool IsCutBeamOn => _cutting.IsBeamOn;
        public Point3D CutPoint => _cutting.CutPoint;
        public double CutProgress => _cutting.Progress;

        public string RadarSummary
        {
            get => _radarSummary;
            private set => SetProperty(ref _radarSummary, value);
        }

        public string RadarReadout
        {
            get => _radarReadout;
            private set => SetProperty(ref _radarReadout, value);
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
            _cutTimer.Stop();
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
                case 4:
                    OnPropertyChanged(nameof(Joint5));
                    OnPropertyChanged(nameof(Joint5Text));
                    break;
            }
        }

        private void SyncJointsFromModel()
        {
            OnPropertyChanged(nameof(Joint1));
            OnPropertyChanged(nameof(Joint2));
            OnPropertyChanged(nameof(Joint3));
            OnPropertyChanged(nameof(Joint4));
            OnPropertyChanged(nameof(Joint5));
            OnPropertyChanged(nameof(Joint1Text));
            OnPropertyChanged(nameof(Joint2Text));
            OnPropertyChanged(nameof(Joint3Text));
            OnPropertyChanged(nameof(Joint4Text));
            OnPropertyChanged(nameof(Joint5Text));
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
            _automation.UpdateAttachedPose();
        }

        private void ResetJoints()
        {
            SetJoint(0, 0);
            SetJoint(1, 0);
            SetJoint(2, 0);
            SetJoint(3, 0);
            SetJoint(4, 0);
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
            TryAttachNearby();
        }

        private void StartManualPick()
        {
            if (_automation.IsRunning)
                StopAutomation();

            StopCutting("automation started");
            SetAutomationDiagnostic(string.Empty);
            _automation.ActionDelay = ParseDelay(ActionDelayText, 0.7);
            _automation.SpeedMultiplier = AutomationSpeed;

            if (!_automation.StartManualPick())
                return;

            ObjectStatus = "Object: picking";
            _automationTimer.Start();
        }

        private void StartManualPlace()
        {
            if (_automation.IsRunning)
                StopAutomation();

            StopCutting("automation started");
            SetAutomationDiagnostic(string.Empty);
            _automation.ActionDelay = ParseDelay(ActionDelayText, 0.7);
            _automation.SpeedMultiplier = AutomationSpeed;

            if (!_automation.StartManualPlace())
                return;

            ObjectStatus = "Object: placing";
            _automationTimer.Start();
        }

        private void TryAttachNearby()
        {
            if (_automation.AttachedItem != null)
                return;

            if (Robot.GripperOpen > 0.15)
            {
                ObjectStatus = "Object: close gripper first";
                return;
            }

            var fk = Robot.ForwardKinematics();

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

            if (closestItem == null || closestDistance > RobotArmModel.PickDistance)
            {
                ObjectStatus = "Object: move it closer to gripper";
                return;
            }

            var inverse = Robot.ToolOrientation(fk);
            inverse.Invert();
            _automation.AttachManually(closestItem, inverse.Transform(closestItem.Position - fk.EePos));

            ObjectStatus = "Object: attached";
            OnPoseChanged();
        }

        private void ReleaseObject()
        {
            _automation.DetachAttached();
            ObjectStatus = "Object: released";
            OnPoseChanged();
        }

        private void StartAutomation()
        {
            if (_automation.IsManualTransfer)
                StopAutomation();

            StopCutting("automation started");
            HideToast();
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

        private void OnActivateTool()
        {
            var mode = (ToolMode)ToolModeIndex;
            ToolActivated?.Invoke(mode);

            if (mode != ToolMode.Cut)
            {
                StopCutting("tool switched");
                return;
            }

            if (_cutCompleted)
            {
                SetAutomationDiagnostic("Cutting finished. Press Reset Auto Objects to repeat.");
                return;
            }

            StartCutting();
        }

        private void StartCutting()
        {
            if (_automation.IsRunning)
                StopAutomation();

            if (_cutting.IsRunning)
                return;

            SetAutomationDiagnostic(string.Empty);
            HideToast();
            _cutting.SpeedMultiplier = AutomationSpeed;

            if (!_cutting.Start())
                return;

            ObjectStatus = "Object: cutting";
            _cutTimer.Start();
        }

        private void StopCutting(string reason)
        {
            _cutTimer.Stop();

            if (_cutting.IsRunning)
                _cutting.Stop(reason);

            OnPropertyChanged(nameof(IsCutting));
            OnPropertyChanged(nameof(IsCutBeamOn));
        }

        private void CuttingTick()
        {
            _cutting.Tick();
            SyncJointsFromModel();
            RefreshRadar();
            OnPropertyChanged(nameof(CutProgress));
            OnPropertyChanged(nameof(IsCutBeamOn));

            if (!_cutting.IsRunning)
                _cutTimer.Stop();
        }

        private void OnCuttingStatusChanged()
        {
            CutStatus = _cutting.Status;
            OnPropertyChanged(nameof(IsCutting));
            OnPropertyChanged(nameof(IsCutBeamOn));
        }

        private void OnCuttingFinished()
        {
            _cutTimer.Stop();
            OnPropertyChanged(nameof(IsCutting));
            OnPropertyChanged(nameof(IsCutBeamOn));

            if (_cutting.ObjectsCut >= _items.Count)
            {
                _cutCompleted = true;
                ShowToast("Cutting operation finished. Press Reset Auto Objects to repeat.");
                ObjectStatus = "Object: cut";
            }
            else
            {
                SetAutomationDiagnostic(_cutting.Status);
            }
        }

        // A finished object leaves the scene and both halves join the item
        // list, so the automation transports them like any other object.
        private void OnItemCut(AutomationItem item, Vector3D normal)
        {
            int index = _items.IndexOf(item);
            if (index < 0)
                return;

            var (keep, offcut) = CutPlanner.Split(item, normal);
            _items.RemoveAt(index);
            _items.Insert(index, keep);
            _items.Insert(index + 1, offcut);
            RefreshRadar();
            AutomationSceneChanged?.Invoke();
        }

        private void ResetAutomationObjects()
        {
            _automationTimer.Stop();
            _automation.Stop("Automation stopped.");

            _cutTimer.Stop();
            _cutting.ResetState();
            _cutCompleted = false;
            HideToast();

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

            SyncJointsFromModel();
            OnPropertyChanged(nameof(WristRoll));
            OnPropertyChanged(nameof(WristRollText));
            OnPropertyChanged(nameof(GripperOpen));
            RefreshRadar();
            AutomationSceneChanged?.Invoke();
        }

        // The radar panel shows exactly what the sensor reports: object count,
        // measured size in centimetres, and world coordinates.
        private void RefreshRadar()
        {
            var detected = _radar.Scan();
            RadarSummary = detected.Count == 0
                ? "no objects detected"
                : $"{detected.Count} object{(detected.Count == 1 ? string.Empty : "s")} detected";

            if (detected.Count == 0)
            {
                RadarReadout = "-";
                return;
            }

            var lines = new List<string>(detected.Count);
            for (int i = 0; i < detected.Count; i++)
            {
                var d = detected[i];
                lines.Add(
                    $"#{i + 1}: {d.HalfSize.X * 200,3:0}x{d.HalfSize.Y * 200,3:0}x{d.HalfSize.Z * 200,3:0} cm" +
                    $"   x {d.Position.X,5:0.00}  y {d.Position.Y,4:0.00}  z {d.Position.Z,5:0.00}");
            }

            RadarReadout = string.Join(Environment.NewLine, lines);
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
            _obstacles.AddRange(ObstacleLayout.Generate(Math.Abs(PickupZone.X), ++_obstacleTestNumber));
            AutomationSceneChanged?.Invoke();
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
            RefreshRadar();

            if (!_automation.IsRunning)
                _automationTimer.Stop();
        }

        private void WaveTick()
        {
            _waveTime += TimerMilliseconds / 1000.0;
            Robot.Angles[3] = 30 * Math.Sin(_waveTime * 2.0);
            Robot.Angles[4] = 20 * Math.Sin(_waveTime * 3.0 + 0.5);
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

            ObjectStatus = _automation.AttachedItem != null ? "Object: attached" : "Object: free";

            if (_obstacleTestActive)
            {
                if (_automation.Diagnostic != "Operation finished")
                {
                    _obstacles.Clear();
                    AutomationSceneChanged?.Invoke();
                }

                _obstacleTestActive = false;
            }

            if (_automation.Diagnostic == "Operation finished")
            {
                SetAutomationDiagnostic(string.Empty);
                ShowToast("Transport operation finished. Press Reset Auto Objects to repeat.");
            }

            OnPropertyChanged(nameof(IsAutomationRunning));
        }

        private void ShowToast(string message)
        {
            ToastMessage = message;
            HasToast = true;
        }

        private void HideToast()
        {
            HasToast = false;
            ToastMessage = string.Empty;
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
