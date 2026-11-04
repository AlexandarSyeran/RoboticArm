using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Media.Animation;
using System.Text.Json;
using Microsoft.Win32;

namespace RoboticArm
{
    public partial class MainWindow : Window
    {
        private Model3DGroup _sceneGroup;

        // models and transforms
        private GeometryModel3D[] _linkModels;
        private MatrixTransform3D[] _linkTransforms;
        private GeometryModel3D _eeSphereModel;
        // gripper
        private GeometryModel3D[] _gripperFingers;
        private int _gripperFingerCount = 4;
        private double _gripperOpen = 0.0; // 0 closed, 1 open
        private AxisAngleRotation3D _wristRollRotation;
        private System.Windows.Threading.DispatcherTimer _wristAutoTimer;
        private double _wristRollAngle = 0.0;
        private enum ToolMode { Grip, Cut, Weld }
        private ToolMode _currentTool = ToolMode.Grip;
        private GeometryModel3D _toolEffectSphere;
        private System.Windows.Threading.DispatcherTimer _toolEffectTimer;

        // Wrist orientation triad and pickable object
        private GeometryModel3D[] _wristAxes;
        private GeometryModel3D _pickableObject;
        private bool _objectAttached;
        private Vector3D _objectToolOffset;
        private const double PickDistance = 0.55;

        // Automated pick-and-place scene and state
        private readonly System.Collections.Generic.List<AutomationItem> _automationItems = new();
        private GeometryModel3D _dropZoneModel;
        private GeometryModel3D _pickupZoneModel;
        private GeometryModel3D _conveyorModel;
        private GeometryModel3D _automationAttachedObject;
        private System.Windows.Threading.DispatcherTimer _automationTimer;
        private AutomationPhase _automationPhase = AutomationPhase.Idle;
        private int _automationIndex;
        private double _automationPhaseTime;
        private double _automationArrivalDelay = 2.0;
        private double _automationActionDelay = 0.7;
        private Point3D _automationPickTarget;
        private Point3D _automationDropTarget;
        private Point3D _defaultPickupZone;
        private readonly double[] _initialFoldedAngles = { 0, -35, 100, -20 };

        // Animation and trajectory playback
        private enum AutomationPhase
        {
            Idle,
            MoveToPick,
            CloseGripper,
            Attach,
            Lift,
            MoveToDrop,
            OpenGripper,
            ReturnHome,
            ArrivalDelay
        }

        // camera spherical
        // Natural front view: camera is in front of the pickup zone and slightly above it.
        private double _camTheta = -Math.PI / 2.0; // front (-Z)
        private double _camPhi = Math.PI * 0.08;   // slight eye-level elevation
        private double _camRadius = 8.0;
        private Point3D _cameraTarget = new Point3D(3.0, 0.4, 0.0);
        private Point _lastMouse;
        private bool _isLeftDown, _isRightDown;

        // link lengths accessible for IK
        private double L1 = 2.0, L2 = 1.5, L3 = 1.0;

        // kinematic state
        private double[] _angles; // degrees [base, shoulder, elbow, wristPitch]
        // joint limits (degrees)
        private double[] _minAngles;
        private double[] _maxAngles;

        // animation
        private System.Windows.Threading.DispatcherTimer _animTimer;
        private double _animT = 0;

        // keyframes with timestamps
        private System.Collections.Generic.List<Keyframe> _keyframes = new System.Collections.Generic.List<Keyframe>();
        private System.Windows.Threading.DispatcherTimer _trajTimer;
        private int _trajIndex = 0;
        private double _trajSegT = 0; // 0..1
        private double _trajTime = 0.0; // playback elapsed seconds
        private bool _isDraggingTarget = false;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            BuildScene();
        }

        private void BuildScene()
        {
            // Camera
            var cam = new PerspectiveCamera();
            MainViewport.Camera = cam;
            UpdateCamera();
            // mouse handlers
            MainViewport.MouseDown += MainViewport_MouseDown;
            MainViewport.MouseUp += MainViewport_MouseUp;
            MainViewport.MouseMove += MainViewport_MouseMove;
            MainViewport.MouseWheel += MainViewport_MouseWheel;

            // Lights
            _sceneGroup = new Model3DGroup();
            _sceneGroup.Children.Add(new AmbientLight(Color.FromRgb(60, 60, 60)));
            _sceneGroup.Children.Add(new DirectionalLight(Colors.White, new Vector3D(-1, -1, -2)));

            // ground grid
            var gridMesh = MeshBuilder.CreateGrid(10, 0.5);
            var gridMat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(40, 40, 40)));
            var gridModel = new GeometryModel3D(gridMesh, gridMat);
            _sceneGroup.Children.Add(gridModel);

            // Base
            double baseWidth = 1.6, baseDepth = 1.6, baseHeight = 0.3;
            var baseMesh = MeshBuilder.CreateBox(baseWidth, baseHeight, baseDepth);
            var baseMat = new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(120, 120, 120)));
            var baseModel = new GeometryModel3D(baseMesh, baseMat);
            baseModel.Transform = new TranslateTransform3D(0, baseHeight / 2.0, 0);
            _sceneGroup.Children.Add(baseModel);

            // prepare arrays
            int n = 4; // base, shoulder, elbow, wristPitch
            _angles = (double[])_initialFoldedAngles.Clone();
            _linkModels = new GeometryModel3D[n];
            _linkTransforms = new MatrixTransform3D[n];

            _minAngles = new double[n];
            _maxAngles = new double[n];
            // Base is centered on the default pickup direction and may rotate only 90 degrees left/right.
            _minAngles[0] = -90; _maxAngles[0] = 90;
            _minAngles[1] = -90; _maxAngles[1] = 90;
            _minAngles[2] = -135; _maxAngles[2] = 135;
            _minAngles[3] = -90; _maxAngles[3] = 90;
            ClampAngles();

            double thickness = 0.28;

            // create links (meshes oriented along X by rotating cylinder)
            var mesh1 = MeshBuilder.CreateCylinder(L1, thickness/2.0, 40);
            var mesh2 = MeshBuilder.CreateCylinder(L2, thickness*0.45, 36);
            var mesh3 = MeshBuilder.CreateCylinder(L3, thickness*0.4, 28);
            var mesh4 = MeshBuilder.CreateCylinder(0.6, thickness*0.35, 20); // wrist small

            var mats = new[] {
                CreateMaterial(Color.FromRgb(200,80,80)),
                CreateMaterial(Color.FromRgb(80,160,200)),
                CreateMaterial(Color.FromRgb(160,200,90)),
                CreateMaterial(Color.FromRgb(200,160,220)) };

            var meshes = new[] { mesh1, mesh2, mesh3, mesh4 };

            for (int i = 0; i < n; i++)
            {
                var gm = new GeometryModel3D(meshes[i], mats[i]);
                // initial transform identity
                var mt = new MatrixTransform3D(Matrix3D.Identity);
                gm.Transform = mt;
                _linkModels[i] = gm;
                _linkTransforms[i] = mt;
                _sceneGroup.Children.Add(gm);

                // joint sphere
                var sphere = new GeometryModel3D(MeshBuilder.CreateSphere(0.12, 12, 8), CreateMaterial(Color.FromRgb(220,220,220)));
                _sceneGroup.Children.Add(sphere);
            }

            // end effector sphere
            _eeSphereModel = new GeometryModel3D(MeshBuilder.CreateSphere(0.14, 12, 8), CreateMaterial(Color.FromRgb(255,200,50)));
            _sceneGroup.Children.Add(_eeSphereModel);

            // gripper fingers
            _gripperFingers = new GeometryModel3D[_gripperFingerCount];
            for (int i = 0; i < _gripperFingerCount; i++)
            {
                var fingerMesh = MeshBuilder.CreateBox(0.4, 0.06, 0.12);
                var fingerMat = CreateMaterial(Color.FromRgb(200, 200, 200));
                var fingerModel = new GeometryModel3D(fingerMesh, fingerMat);
                fingerModel.Transform = new MatrixTransform3D(Matrix3D.Identity);
                _gripperFingers[i] = fingerModel;
                _sceneGroup.Children.Add(fingerModel);
            }

            // tool effect sphere (hidden until active)
            _toolEffectSphere = new GeometryModel3D(MeshBuilder.CreateSphere(0.08, 12, 8), CreateMaterial(Color.FromRgb(255, 50, 50)));
            _toolEffectSphere.Transform = new TranslateTransform3D(0, -100, 0); // hide offscene
            _sceneGroup.Children.Add(_toolEffectSphere);

            // Wrist/tool orientation triad. Meshes are cylinders along local Y.
            _wristAxes = new[]
            {
                CreateAxisModel(Colors.Red),
                CreateAxisModel(Colors.Green),
                CreateAxisModel(Colors.Blue)
            };
            foreach (var axis in _wristAxes)
                _sceneGroup.Children.Add(axis);

            // Sample object used by the pick-and-attach demonstration.
            _pickableObject = new GeometryModel3D(
                MeshBuilder.CreateBox(0.32, 0.32, 0.32),
                CreateMaterial(Color.FromRgb(245, 170, 45)));
            _sceneGroup.Children.Add(_pickableObject);
            PlaceObjectNearTool();

            var modelVisual = new ModelVisual3D { Content = _sceneGroup };
            MainViewport.Children.Add(modelVisual);

            // set initial UI text
            Joint1Value.Text = "0°";
            Joint2Value.Text = "0°";
            Joint3Value.Text = "0°";
            WristPitchValue.Text = "0°";
            WristRollValue.Text = "0°";

            // The folded initial pose defines the automatic pickup direction and workspace.
            _defaultPickupZone = ComputeForwardKinematics(_angles).eePos;
            _automationDropTarget = new Point3D(
                _defaultPickupZone.X,
                _defaultPickupZone.Y,
                _defaultPickupZone.Z + 2.4);
            _cameraTarget = _defaultPickupZone;
            UpdateCamera();
            BuildAutomationScene();
            ApplyFkToModels();

            // animation timer
            _animTimer = new System.Windows.Threading.DispatcherTimer();
            _animTimer.Interval = TimeSpan.FromMilliseconds(30);
            _animTimer.Tick += AnimTimer_Tick;

            // wrist auto-rotate timer
            _wristAutoTimer = new System.Windows.Threading.DispatcherTimer();
            _wristAutoTimer.Interval = TimeSpan.FromMilliseconds(30);
            _wristAutoTimer.Tick += (s, ev) => {
                _wristRollAngle += 2.5; if (_wristRollAngle >= 360) _wristRollAngle -= 360;
                WristRollSlider.Value = _wristRollAngle;
            };

            // tool effect timer
            _toolEffectTimer = new System.Windows.Threading.DispatcherTimer();
            _toolEffectTimer.Interval = TimeSpan.FromMilliseconds(100);
            int flash = 0;
            _toolEffectTimer.Tick += (s, ev) => {
                flash++;
                var mat = _toolEffectSphere.Material as MaterialGroup;
                if (mat != null)
                {
                    var diff = mat.Children[0] as DiffuseMaterial;
                    if (_currentTool == ToolMode.Cut) diff.Brush = new SolidColorBrush(Color.FromRgb((byte)(255), (byte)(50 + (flash%2)*100), 50));
                    else if (_currentTool == ToolMode.Weld) diff.Brush = new SolidColorBrush(Color.FromRgb(255, (byte)(150 + (flash%2)*50), 0));
                }
                if (flash > 8) { _toolEffectTimer.Stop(); _toolEffectSphere.Transform = new TranslateTransform3D(0, -100, 0); }
            };

            // trajectory timer
            _trajTimer = new System.Windows.Threading.DispatcherTimer();
            _trajTimer.Interval = TimeSpan.FromMilliseconds(30);
            _trajTimer.Tick += TrajTimer_Tick;

            _automationTimer = new System.Windows.Threading.DispatcherTimer();
            _automationTimer.Interval = TimeSpan.FromMilliseconds(40);
            _automationTimer.Tick += AutomationTimer_Tick;

        }
        private Material CreateMaterial(Color color)
        {
            var mg = new MaterialGroup();
            mg.Children.Add(new DiffuseMaterial(new SolidColorBrush(color)));
            mg.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.White), 32));
            return mg;
        }

        private GeometryModel3D CreateAxisModel(Color color)
        {
            return new GeometryModel3D(
                MeshBuilder.CreateCylinder(0.55, 0.025, 16),
                CreateMaterial(color));
        }
        
        private static Quaternion RotationFromTo(Vector3D from, Vector3D to)
        {
            from.Normalize();
            to.Normalize();
            double dot = Math.Max(-1.0, Math.Min(1.0, Vector3D.DotProduct(from, to)));
            if (dot > 0.999999)
                return Quaternion.Identity;

            if (dot < -0.999999)
            {
                Vector3D perpendicular = Vector3D.CrossProduct(from, new Vector3D(1, 0, 0));
                if (perpendicular.Length < 1e-6)
                    perpendicular = Vector3D.CrossProduct(from, new Vector3D(0, 1, 0));
                perpendicular.Normalize();
                return new Quaternion(perpendicular, 180);
            }

            var axis = Vector3D.CrossProduct(from, to);
            axis.Normalize();
            return new Quaternion(axis, Math.Acos(dot) * 180.0 / Math.PI);
        }

        private void UpdateWristAxes(Point3D origin, Matrix3D orientation)
        {
            if (_wristAxes == null) return;

            var localAxes = new[]
            {
                new Vector3D(1, 0, 0),
                new Vector3D(0, 1, 0),
                new Vector3D(0, 0, 1)
            };

            for (int i = 0; i < _wristAxes.Length; i++)
            {
                var direction = orientation.Transform(localAxes[i]);
                direction.Normalize();
                var midpoint = new Point3D(
                    origin.X + direction.X * 0.275,
                    origin.Y + direction.Y * 0.275,
                    origin.Z + direction.Z * 0.275);

                var transform = new Transform3DGroup();
                transform.Children.Add(new RotateTransform3D(
                    new QuaternionRotation3D(RotationFromTo(new Vector3D(0, 1, 0), direction))));
                transform.Children.Add(new TranslateTransform3D(midpoint.X, midpoint.Y, midpoint.Z));
                _wristAxes[i].Transform = transform;
            }
        }

        private void PlaceObjectNearTool()
        {
            if (_pickableObject == null) return;
            var fk = ComputeForwardKinematics(_angles ?? new double[4]);
            _objectAttached = false;
            _objectToolOffset = new Vector3D(0, 0, 0);
            _pickableObject.Transform = new TranslateTransform3D(fk.eePos.X, fk.eePos.Y, fk.eePos.Z);
            if (ObjectStatusText != null) ObjectStatusText.Text = "Object: free";
        }

        private void BuildAutomationScene()
        {
            // Conveyor surface and a green destination zone. Y is the vertical axis in this scene.
            _conveyorModel = new GeometryModel3D(
                MeshBuilder.CreateBox(4.8, 0.12, 2.4),
                CreateMaterial(Color.FromRgb(65, 70, 78)));
            _conveyorModel.Transform = new TranslateTransform3D(_defaultPickupZone.X, 0.06, _defaultPickupZone.Z + 0.65);
            _sceneGroup.Children.Add(_conveyorModel);

            _pickupZoneModel = new GeometryModel3D(
                MeshBuilder.CreateBox(0.9, 0.025, 0.9),
                CreateMaterial(Color.FromRgb(50, 125, 210)));
            _pickupZoneModel.Transform = new TranslateTransform3D(
                _defaultPickupZone.X, 0.13, _defaultPickupZone.Z);
            _sceneGroup.Children.Add(_pickupZoneModel);

            _dropZoneModel = new GeometryModel3D(
                MeshBuilder.CreateBox(1.0, 0.025, 1.0),
                CreateMaterial(Color.FromRgb(40, 170, 90)));
            _dropZoneModel.Transform = new TranslateTransform3D(
                _automationDropTarget.X, 0.13, _automationDropTarget.Z);
            _sceneGroup.Children.Add(_dropZoneModel);

            ResetAutomationObjects();
        }

        // Reset the automation scene: remove all objects, reset robot state,
        // and place new objects at the pickup zone.
        private void ResetAutomationObjects()
        {
            StopAutomation();
            foreach (var item in _automationItems)
                _sceneGroup.Children.Remove(item.Model);
            _automationItems.Clear();

            var positions = new[]
            {
                new Point3D(_defaultPickupZone.X - 0.45, _defaultPickupZone.Y, _defaultPickupZone.Z - 0.35),
                new Point3D(_defaultPickupZone.X + 0.45, _defaultPickupZone.Y, _defaultPickupZone.Z - 0.35),
                new Point3D(_defaultPickupZone.X - 0.45, _defaultPickupZone.Y, _defaultPickupZone.Z + 0.35),
                new Point3D(_defaultPickupZone.X + 0.45, _defaultPickupZone.Y, _defaultPickupZone.Z + 0.35)
            };
            var colors = new[]
            {
                Color.FromRgb(235, 80, 70),
                Color.FromRgb(70, 150, 235),
                Color.FromRgb(240, 190, 55),
                Color.FromRgb(170, 90, 220)
            };

            for (int i = 0; i < positions.Length; i++)
            {
                var model = new GeometryModel3D(
                    MeshBuilder.CreateBox(0.32, 0.32, 0.32),
                    CreateMaterial(colors[i]));
                model.Transform = new TranslateTransform3D(
                    positions[i].X, positions[i].Y, positions[i].Z);
                _sceneGroup.Children.Add(model);
                _automationItems.Add(new AutomationItem
                {
                    Model = model,
                    StartPosition = positions[i]
                });
            }

            _automationAttachedObject = null;
            _automationIndex = 0;
            _automationPhase = AutomationPhase.Idle;
            Array.Copy(_initialFoldedAngles, _angles, _initialFoldedAngles.Length);
            ClampAngles();
            _wristRollAngle = 0;
            _gripperOpen = 1;
            if (Joint1Slider != null) Joint1Slider.Value = _angles[0];
            if (Joint2Slider != null) Joint2Slider.Value = _angles[1];
            if (Joint3Slider != null) Joint3Slider.Value = _angles[2];
            if (WristPitchSlider != null) WristPitchSlider.Value = _angles[3];
            if (WristRollSlider != null) WristRollSlider.Value = 0;
            if (GripperOpenSlider != null) GripperOpenSlider.Value = 1;
            if (AutomationStatusText != null) AutomationStatusText.Text = "Automation: idle";
            ApplyFkToModels();
        }
        // Attempt to attach the automation object to the end effector
        // if within range and gripper is closed.
        private bool TryAttachAutomationObject(AutomationItem item)
        {
            if (item == null || _gripperOpen > 0.15) return false;
            var fk = ComputeForwardKinematics(_angles);
            var objectPosition = item.Model.Transform.Transform(new Point3D(0, 0, 0));
            var distance = objectPosition - fk.eePos;
            if (distance.Length > PickDistance) return false;

            var toolOrientation = fk.eeRot *
                ForwardKinematics.RotateAxisAngle(new Vector3D(1, 0, 0), _wristRollAngle);
            var inverse = toolOrientation;
            inverse.Invert();
            _objectToolOffset = inverse.Transform(distance);
            _automationAttachedObject = item.Model;
            return true;
        }

        // Release the attached automation object and place it at the specified position
        private void ReleaseAutomationObject(Point3D position)
        {
            if (_automationAttachedObject == null) return;
            _automationAttachedObject.Transform = new TranslateTransform3D(
                position.X, position.Y, position.Z);
            _automationAttachedObject = null;
            _objectToolOffset = new Vector3D(0, 0, 0);
        }

        // Solve the inverse kinematics for the automation target position, updating joint angles.
        private bool SolveAutomationPosition(Point3D target, int iterations = 8)
        {
            if (_angles == null) return false;
            var current = ComputeForwardKinematics(_angles);
            var solved = InverseKinematics6D.Solve6D(
                target,
                current.eeRot,
                _angles,
                angles =>
                {
                    var fk = ComputeForwardKinematics(angles);
                    return (fk.eePos, fk.eeRot, fk.jointOrigins, fk.jointAxes);
                },
                _minAngles,
                _maxAngles,
                false,
                0.08,
                1.0,
                iterations,
                0.12);

            ClampAngles();
            ApplyFkToModels();
            return solved;
        }
        // Get the world position of the automation object based on its model transform.
        private Point3D GetAutomationObjectPosition(AutomationItem item)
        {
            return item.Model.Transform.Transform(new Point3D(0, 0, 0));
        }
        // Automation timer tick: handles the state machine for automated pick-and-place,
        // including moving to pick, closing gripper, attaching object, lifting, moving to drop, opening gripper,
        // and returning home.
        private void AutomationTimer_Tick(object sender, EventArgs e)
        {
            if (_automationPhase == AutomationPhase.Idle || _automationItems.Count == 0)
                return;

            _automationPhaseTime += _automationTimer.Interval.TotalSeconds;
            var item = _automationItems[_automationIndex];
            var objectPosition = GetAutomationObjectPosition(item);
            var fk = ComputeForwardKinematics(_angles);

            switch (_automationPhase)
            {
                case AutomationPhase.MoveToPick:
                    _automationPickTarget = new Point3D(
                        objectPosition.X, objectPosition.Y + 0.28, objectPosition.Z);
                    SolveAutomationPosition(_automationPickTarget);
                    AutomationStatusText.Text = $"Automation: moving to object {_automationIndex + 1}/{_automationItems.Count}";
                    if ((ComputeForwardKinematics(_angles).eePos - _automationPickTarget).Length < 0.16)
                    {
                        _automationPhase = AutomationPhase.CloseGripper;
                        _automationPhaseTime = 0;
                    }
                    break;

                case AutomationPhase.CloseGripper:
                    _gripperOpen = 0;
                    GripperOpenSlider.Value = 0;
                    ApplyFkToModels();
                    AutomationStatusText.Text = "Automation: closing gripper";
                    if (_automationPhaseTime >= _automationActionDelay)
                    {
                        _automationPhase = AutomationPhase.Attach;
                        _automationPhaseTime = 0;
                    }
                    break;

                case AutomationPhase.Attach:
                    if (!TryAttachAutomationObject(item))
                    {
                        _automationPhase = AutomationPhase.MoveToPick;
                        _automationPhaseTime = 0;
                        break;
                    }
                    AutomationStatusText.Text = "Automation: object attached";
                    _automationPhase = AutomationPhase.Lift;
                    _automationPhaseTime = 0;
                    break;

                case AutomationPhase.Lift:
                    var liftTarget = new Point3D(
                        _automationPickTarget.X, _automationPickTarget.Y + 0.85, _automationPickTarget.Z);
                    SolveAutomationPosition(liftTarget);
                    AutomationStatusText.Text = "Automation: lifting";
                    if ((ComputeForwardKinematics(_angles).eePos - liftTarget).Length < 0.18)
                    {
                        _automationPhase = AutomationPhase.MoveToDrop;
                        _automationPhaseTime = 0;
                    }
                    break;

                case AutomationPhase.MoveToDrop:
                    var dropApproach = new Point3D(
                        _automationDropTarget.X, _automationDropTarget.Y + 0.45, _automationDropTarget.Z);
                    SolveAutomationPosition(dropApproach);
                    AutomationStatusText.Text = "Automation: moving to drop zone";
                    if ((ComputeForwardKinematics(_angles).eePos - dropApproach).Length < 0.18)
                    {
                        _automationPhase = AutomationPhase.OpenGripper;
                        _automationPhaseTime = 0;
                    }
                    break;

                case AutomationPhase.OpenGripper:
                    _gripperOpen = 1;
                    GripperOpenSlider.Value = 1;
                    ApplyFkToModels();
                    AutomationStatusText.Text = "Automation: releasing object";
                    if (_automationPhaseTime >= _automationActionDelay)
                    {
                        ReleaseAutomationObject(_automationDropTarget);
                        item.Completed = true;
                        _automationPhase = AutomationPhase.ReturnHome;
                        _automationPhaseTime = 0;
                    }
                    break;

                case AutomationPhase.ReturnHome:
                    var home = _defaultPickupZone;
                    SolveAutomationPosition(home);
                    AutomationStatusText.Text = "Automation: returning for next object";
                    if ((ComputeForwardKinematics(_angles).eePos - home).Length < 0.2)
                    {
                        _automationPhase = AutomationPhase.ArrivalDelay;
                        _automationPhaseTime = 0;
                    }
                    break;

                case AutomationPhase.ArrivalDelay:
                    AutomationStatusText.Text = $"Automation: waiting {_automationArrivalDelay - _automationPhaseTime:0.0}s";
                    if (_automationPhaseTime >= _automationArrivalDelay)
                    {
                        _automationIndex++;
                        if (_automationIndex >= _automationItems.Count)
                        {
                            StopAutomation();
                            AutomationStatusText.Text = "Automation: completed";
                        }
                        else
                        {
                            _automationPhase = AutomationPhase.MoveToPick;
                            _automationPhaseTime = 0;
                        }
                    }
                    break;
            }
        }
        // Update camera position based on spherical coordinates, looking at _cameraTarget
        private void UpdateCamera()
        {
            if (!(MainViewport.Camera is PerspectiveCamera pc)) return;
            double x = _camRadius * Math.Cos(_camTheta) * Math.Cos(_camPhi);
            double z = _camRadius * Math.Sin(_camTheta) * Math.Cos(_camPhi);
            double y = _camRadius * Math.Sin(_camPhi);
            var pos = new Point3D(
                _cameraTarget.X + x,
                _cameraTarget.Y + y,
                _cameraTarget.Z + z);
            pc.Position = pos;
            pc.LookDirection = new Vector3D(
                _cameraTarget.X - pos.X,
                _cameraTarget.Y - pos.Y,
                _cameraTarget.Z - pos.Z);
            pc.UpDirection = new Vector3D(0, 1, 0);
            pc.FieldOfView = 45;
        }

        // Mouse event handlers for camera control
        private void MainViewport_MouseDown(object sender, MouseButtonEventArgs e)
        {
            _lastMouse = e.GetPosition(this);
            if (e.ChangedButton == MouseButton.Left) _isLeftDown = true;
            if (e.ChangedButton == MouseButton.Right) _isRightDown = true;
            Mouse.Capture(MainViewport);
        }

        private void MainViewport_MouseUp(object sender, MouseButtonEventArgs e)
        {
            if (e.ChangedButton == MouseButton.Left) _isLeftDown = false;
            if (e.ChangedButton == MouseButton.Right) _isRightDown = false;
            if (_isDraggingTarget && e.ChangedButton == MouseButton.Left)
            {
                _isDraggingTarget = false;
                MainViewport.ReleaseMouseCapture();
            }
            else
            {
                Mouse.Capture(null);
            }
        }

        private void MainViewport_MouseMove(object sender, MouseEventArgs e)
        {
            var p = e.GetPosition(this);
            var dx = p.X - _lastMouse.X;
            var dy = p.Y - _lastMouse.Y;
            _lastMouse = p;
            if (_isLeftDown)
            {
                _camTheta += dx * 0.01;
                _camPhi -= dy * 0.01;
                _camPhi = Math.Max(-Math.PI/2 + 0.01, Math.Min(Math.PI/2 - 0.01, _camPhi));
                UpdateCamera();
            }
            else if (_isRightDown)
            {
                // simple zoom-in/out with right drag vertical
                _camRadius += dy * 0.02;
                _camRadius = Math.Max(1.0, _camRadius);
                UpdateCamera();
            }
        }

        // Compute FK: return end-effector position and fill joint origins/axes
        // local joint axes in joint local frames: joint0 axis (0,0,1), joint1 axis (0,1,0), joint2 axis (0,1,0), joint3 axis (0,1,0)
        private (Point3D eePos, Matrix3D eeRot, Point3D[] jointOrigins, Vector3D[] jointAxes) ComputeForwardKinematics(double[] angles)
        {
            int n = angles.Length;
            var jointOrigins = new Point3D[n];
            var jointAxes = new Vector3D[n];

            // base origin
            double baseHeight = 0.3;
            Matrix3D M = ForwardKinematics.Translate(0, baseHeight / 2.0, 0);

            double[] L = new double[] { L1, L2, L3, 0.6 };

            Vector3D[] localAxes = new Vector3D[] {
                new Vector3D(0,1,0), // base yaw: vertical axis
                new Vector3D(0,0,1), // shoulder pitch
                new Vector3D(0,0,1), // elbow pitch
                new Vector3D(0,0,1)  // wrist pitch
            };

            for (int i = 0; i < n; i++)
            {
                // joint origin
                jointOrigins[i] = M.Transform(new Point3D(0, 0, 0));
                // axis in world coords (transform vector)
                var axisWorld = M.Transform(localAxes[i]);
                axisWorld.Normalize();
                jointAxes[i] = axisWorld;

                // apply rotation about axisWorld
                var R = ForwardKinematics.RotateAxisAngle(axisWorld, angles[i]);
                M = R * M;

                // translate along local X by Li
                var T = ForwardKinematics.Translate(L[i], 0, 0);
                M = T * M;
            }

            var ee = M.Transform(new Point3D(0, 0, 0));
            // rotation part is M with translation zeros
            var rot = M;
            rot.OffsetX = rot.OffsetY = rot.OffsetZ = 0;
            return (ee, rot, jointOrigins, jointAxes);
        }
        // Apply FK results to models: update transforms of link models,
        // end effector, and gripper fingers.
        private void ApplyFkToModels()
        {
            // compute transforms and set MatrixTransform3D for each link model and end effector
            var fk = ComputeForwardKinematics(_angles);
            var jointOrigins = fk.jointOrigins;

            // compute M progressively to get center transforms
            Matrix3D M = ForwardKinematics.Translate(0, 0.3 / 2.0, 0);
            Vector3D[] localAxes = new Vector3D[] {
                new Vector3D(0,1,0),
                new Vector3D(0,0,1),
                new Vector3D(0,0,1),
                new Vector3D(0,0,1)
            };
            double[] L = new double[] { L1, L2, L3, 0.6 };

            for (int i = 0; i < _linkModels.Length; i++)
            {
                // apply rotation about axis
                var axisWorld = M.Transform(localAxes[i]); axisWorld.Normalize();
                var R = ForwardKinematics.RotateAxisAngle(axisWorld, _angles[i]);
                M = R * M;
                // center offset
                var Tc = ForwardKinematics.Translate(L[i] / 2.0, 0, 0);
                var Mcenter = Tc * M;
                // align cylinder (created along Y) to X by rotating -90 around Z in model local space
                var Ralign = ForwardKinematics.RotateAxisAngle(new Vector3D(0,0,1), -90);
                var final = Mcenter;
                // apply alignment after center transform: final = final * inv? We'll multiply on left by Ralign to rotate model local axes
                final = Ralign * final;
                var mat = final;
                _linkTransforms[i].Matrix = mat;

                // translate to next joint origin
                var T = ForwardKinematics.Translate(L[i], 0, 0);
                M = T * M;
            }

            // set end effector sphere position
            var eePos = fk.eePos;
            _eeSphereModel.Transform = new TranslateTransform3D(eePos.X, eePos.Y, eePos.Z);

            var wristRollMatrix = ForwardKinematics.RotateAxisAngle(new Vector3D(1, 0, 0), _wristRollAngle);
            var toolOrientation = fk.eeRot * wristRollMatrix;
            UpdateWristAxes(eePos, toolOrientation);

            // gripper: position fingers relative to end-effector orientation
            // finger local center offsets (in end-effector local frame)
            double fingerLen = 0.4;
            double fingerRad = 0.12 + 0.06 * _gripperOpen; // gap increases with open
            for (int i = 0; i < _gripperFingerCount; i++)
            {
                double angle = i * Math.PI / 2.0; // around X axis, in YZ plane
                // center offset: forward along X by fingerLen/2 + 0.05, radial offset in Y/Z
                var centerLocal = new Vector3D(fingerLen / 2.0 + 0.06, Math.Cos(angle) * fingerRad, Math.Sin(angle) * fingerRad);
                // wrist roll local rotation matrix
                var Rlocal = wristRollMatrix;
                // total rotation: end-effector rotation * local wrist roll
                var Rtotal = fk.eeRot * Rlocal;
                // compute center in world: transform local vector then add ee position
                var centerWorldVec = Rtotal.Transform(centerLocal);
                var centerWorld = new Point3D(eePos.X + centerWorldVec.X, eePos.Y + centerWorldVec.Y, eePos.Z + centerWorldVec.Z);

                // create rotation quaternion from Rtotal
                var q = MatrixToQuaternion(Rtotal);
                var rot = new RotateTransform3D(new QuaternionRotation3D(q));
                var trans = new TranslateTransform3D(centerWorld.X, centerWorld.Y, centerWorld.Z);
                var tg = new Transform3DGroup();
                tg.Children.Add(rot);
                tg.Children.Add(trans);
                _gripperFingers[i].Transform = tg;
            }

            // tool effect sphere position at end effector tip
            _toolEffectSphere.Transform = new TranslateTransform3D(eePos.X, eePos.Y, eePos.Z + 0.02);

            if (_objectAttached && _pickableObject != null)
            {
                var objectWorld = toolOrientation.Transform(_objectToolOffset);
                _pickableObject.Transform = new TranslateTransform3D(
                    eePos.X + objectWorld.X,
                    eePos.Y + objectWorld.Y,
                    eePos.Z + objectWorld.Z);
            }

            if (_automationAttachedObject != null)
            {
                var objectWorld = toolOrientation.Transform(_objectToolOffset);
                _automationAttachedObject.Transform = new TranslateTransform3D(
                    eePos.X + objectWorld.X,
                    eePos.Y + objectWorld.Y,
                    eePos.Z + objectWorld.Z);
            }
        }

        // Apply IK results to models (placeholder)
        private void ApplyIkToModels()
        {
            // Currently this mirrors ApplyFkToModels: update sliders from _angles and refresh transforms.
            if (_angles == null) return;
            Joint1Slider.Value = _angles[0];
            Joint2Slider.Value = _angles[1];
            Joint3Slider.Value = _angles[2];
            WristPitchSlider.Value = _angles[3];
            ApplyFkToModels();
        }
        // Clamp joint angles to their specified limits

        private void ClampAngles()
        {
            if (_angles == null) return;
            for (int i = 0; i < _angles.Length; i++)
            {
                if (_minAngles != null) _angles[i] = Math.Max(_minAngles[i], _angles[i]);
                if (_maxAngles != null) _angles[i] = Math.Min(_maxAngles[i], _angles[i]);
            }
        }

        // Mouse wheel zoom handler
        private void MainViewport_MouseWheel(object sender, MouseWheelEventArgs e)
        {
            _camRadius *= (e.Delta > 0) ? 0.9 : 1.1;
            _camRadius = Math.Max(1.0, _camRadius);
            UpdateCamera();
        }
        private void Joint1Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_angles == null) return;
            _angles[0] = e.NewValue;
            Joint1Value.Text = $"{Math.Round(e.NewValue, 1)}°";
            ApplyFkToModels();
        }

        private void Joint2Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_angles == null) return;
            _angles[1] = e.NewValue;
            Joint2Value.Text = $"{Math.Round(e.NewValue, 1)}°";
            ApplyFkToModels();
        }

        private void Joint3Slider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_angles == null) return;
            _angles[2] = e.NewValue;
            Joint3Value.Text = $"{Math.Round(e.NewValue, 1)}°";
            ApplyFkToModels();
        }

        private void WristPitchSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (_angles == null) return;
            _angles[3] = e.NewValue;
            WristPitchValue.Text = $"{Math.Round(e.NewValue, 1)}°";
            ApplyFkToModels();
        }

        private void WristRollSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            _wristRollAngle = e.NewValue;
            WristRollValue.Text = $"{Math.Round(e.NewValue, 1)}°";
            // update gripper transforms
            ApplyFkToModels();
        }

        private void ResetButton_Click(object sender, RoutedEventArgs e)
        {
            Joint1Slider.Value = 0;
            Joint2Slider.Value = 0;
            Joint3Slider.Value = 0;
            WristPitchSlider.Value = 0;
            WristRollSlider.Value = 0;
        }

        // 3D IK solver (Damped Least Squares) implemented inline using ComputeForwardKinematics
        private void Solve6DIk_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(IkXText.Text, out double tx)) { MessageBox.Show("Invalid X"); return; }
            if (!double.TryParse(IkYText.Text, out double ty)) { MessageBox.Show("Invalid Y"); return; }
            if (!double.TryParse(IkZText.Text, out double tz)) { MessageBox.Show("Invalid Z"); return; }
            if (!double.TryParse(IkRxText.Text, out double rx)) { MessageBox.Show("Invalid Rx"); return; }
            if (!double.TryParse(IkRyText.Text, out double ry)) { MessageBox.Show("Invalid Ry"); return; }
            if (!double.TryParse(IkRzText.Text, out double rz)) { MessageBox.Show("Invalid Rz"); return; }

            var targetPos = new Point3D(tx, ty, tz);
            // build rotation matrix from Euler angles (degrees) applying Z (rz), Y (ry), X (rx)
            Matrix3D Rz = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), rz);
            Matrix3D Ry = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 1, 0), ry);
            Matrix3D Rx = ForwardKinematics.RotateAxisAngle(new Vector3D(1, 0, 0), rx);
            var targetRot = Rx * Ry * Rz;

            bool useOri = UseOrientationCheck.IsChecked == true;
            bool ok = InverseKinematics6D.Solve6D(targetPos, targetRot, _angles, (angles) =>
            {
                var fk = ComputeForwardKinematics(angles);
                return (fk.eePos, fk.eeRot, fk.jointOrigins, fk.jointAxes);
            }, _minAngles, _maxAngles, useOri, 1e-3, 1e-2, 120, 0.08);

            // clamp to limits
            ClampAngles();

            // update UI
            Joint1Slider.Value = _angles[0];
            Joint2Slider.Value = _angles[1];
            Joint3Slider.Value = _angles[2];
            WristPitchSlider.Value = _angles[3];
            ApplyFkToModels();
        }

        // small 3x3 solver used by IK
        private double[] Solve3x3(double[,] A, double[] b)
        {
            double a11 = A[0, 0], a12 = A[0, 1], a13 = A[0, 2];
            double a21 = A[1, 0], a22 = A[1, 1], a23 = A[1, 2];
            double a31 = A[2, 0], a32 = A[2, 1], a33 = A[2, 2];
            double det = a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
            if (Math.Abs(det) < 1e-9) return null;
            double invDet = 1.0 / det;
            double d1 = b[0], d2 = b[1], d3 = b[2];
            double dx = d1 * (a22 * a33 - a23 * a32) - a12 * (d2 * a33 - a23 * d3) + a13 * (d2 * a32 - a22 * d3);
            double dy = a11 * (d2 * a33 - a23 * d3) - d1 * (a21 * a33 - a23 * a31) + a13 * (a21 * d3 - d2 * a31);
            double dz = a11 * (a22 * d3 - d2 * a32) - a12 * (a21 * d3 - d2 * a31) + d1 * (a21 * a32 - a22 * a31);
            return new double[] { dx * invDet, dy * invDet, dz * invDet };
        }
        // Convert a 3x3 rotation matrix to a quaternion, assuming no scaling or shearing.
        private Quaternion MatrixToQuaternion(Matrix3D m)
        {
            // Convert rotation matrix to quaternion (assuming no scaling)
            double trace = m.M11 + m.M22 + m.M33;
            double w, x, y, z;
            if (trace > 0)
            {
                double s = 0.5 / Math.Sqrt(trace + 1.0);
                w = 0.25 / s;
                x = (m.M32 - m.M23) * s;
                y = (m.M13 - m.M31) * s;
                z = (m.M21 - m.M12) * s;
            }
            else
            {
                if (m.M11 > m.M22 && m.M11 > m.M33)
                {
                    double s = 2.0 * Math.Sqrt(1.0 + m.M11 - m.M22 - m.M33);
                    w = (m.M32 - m.M23) / s;
                    x = 0.25 * s;
                    y = (m.M12 + m.M21) / s;
                    z = (m.M13 + m.M31) / s;
                }
                else if (m.M22 > m.M33)
                {
                    double s = 2.0 * Math.Sqrt(1.0 + m.M22 - m.M11 - m.M33);
                    w = (m.M13 - m.M31) / s;
                    x = (m.M12 + m.M21) / s;
                    y = 0.25 * s;
                    z = (m.M23 + m.M32) / s;
                }
                else
                {
                    double s = 2.0 * Math.Sqrt(1.0 + m.M33 - m.M11 - m.M22);
                    w = (m.M21 - m.M12) / s;
                    x = (m.M13 + m.M31) / s;
                    y = (m.M23 + m.M32) / s;
                    z = 0.25 * s;
                }
            }
            var q = new Quaternion(x, y, z, w);
            q.Normalize();
            return q;
        }

        private void PlayWave_Click(object sender, RoutedEventArgs e)
        {
            _animT = 0;
            _animTimer.Start();
        }

        private void StopWave_Click(object sender, RoutedEventArgs e)
        {
            _animTimer.Stop();
        }

        private void GripperOpenSlider_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            _gripperOpen = e.NewValue;
            ApplyFkToModels();
        }

        private void OpenGripper_Click(object sender, RoutedEventArgs e)
        {
            _gripperOpen = 1.0;
            GripperOpenSlider.Value = _gripperOpen;
            ApplyFkToModels();
        }

        private void CloseGripper_Click(object sender, RoutedEventArgs e)
        {
            _gripperOpen = 0.0;
            GripperOpenSlider.Value = _gripperOpen;
            ApplyFkToModels();
            TryPickObject();
        }

        private void PlaceObject_Click(object sender, RoutedEventArgs e)
        {
            PlaceObjectNearTool();
            ApplyFkToModels();
        }

        private void PickObject_Click(object sender, RoutedEventArgs e)
        {
            TryPickObject();
        }

        private void ReleaseObject_Click(object sender, RoutedEventArgs e)
        {
            _objectAttached = false;
            if (_automationAttachedObject != null)
            {
                _automationAttachedObject = null;
            }
            _objectToolOffset = new Vector3D(0, 0, 0);
            if (ObjectStatusText != null) ObjectStatusText.Text = "Object: released";
            ApplyFkToModels();
        }
        // Start the automation process: pick and place objects from the conveyor to the drop zone.
        private void StartAutomation_Click(object sender, RoutedEventArgs e)
        {
            if (!double.TryParse(ArrivalDelayText.Text, out _automationArrivalDelay) || _automationArrivalDelay < 0)
                _automationArrivalDelay = 2.0;
            if (!double.TryParse(ActionDelayText.Text, out _automationActionDelay) || _automationActionDelay < 0)
                _automationActionDelay = 0.7;

            if (_automationItems.Count == 0)
            {
                AutomationStatusText.Text = "Automation: no objects";
                return;
            }

            _automationIndex = 0;
            while (_automationIndex < _automationItems.Count && _automationItems[_automationIndex].Completed)
                _automationIndex++;
            if (_automationIndex >= _automationItems.Count)
            {
                AutomationStatusText.Text = "Automation: all objects completed";
                return;
            }

            _automationPhase = AutomationPhase.MoveToPick;
            _automationPhaseTime = 0;
            _automationTimer.Start();
        }

        private void StopAutomation_Click(object sender, RoutedEventArgs e)
        {
            StopAutomation();
        }

        private void StopAutomation()
        {
            _automationTimer?.Stop();
            if (_automationPhase != AutomationPhase.Idle)
                _automationPhase = AutomationPhase.Idle;
            if (AutomationStatusText != null)
                AutomationStatusText.Text = "Automation: stopped";
        }

        private void ResetAutomation_Click(object sender, RoutedEventArgs e)
        {
            ResetAutomationObjects();
            _gripperOpen = 1;
            GripperOpenSlider.Value = 1;
            ApplyFkToModels();
        }

        private void ShowWristAxes_Checked(object sender, RoutedEventArgs e)
        {
            if (_wristAxes == null) return;
            bool visible = ShowWristAxesCheck.IsChecked == true;
            foreach (var axis in _wristAxes)
            {
                if (visible)
                {
                    if (!_sceneGroup.Children.Contains(axis))
                        _sceneGroup.Children.Add(axis);
                }
                else
                {
                    _sceneGroup.Children.Remove(axis);
                }
            }
        }
        // Attempt to pick up an object if the gripper is closed and within range.
        private void TryPickObject()
        {
            if (_objectAttached || _automationAttachedObject != null) return;
            if (_gripperOpen > 0.15)
            {
                if (ObjectStatusText != null) ObjectStatusText.Text = "Object: close gripper first";
                return;
            }

            var fk = ComputeForwardKinematics(_angles);
            var wristRollMatrix = ForwardKinematics.RotateAxisAngle(new Vector3D(1, 0, 0), _wristRollAngle);
            var toolOrientation = fk.eeRot * wristRollMatrix;

            GeometryModel3D closestModel = null;
            AutomationItem closestItem = null;
            double closestDistance = double.MaxValue;

            // Prefer the nearest available object from the automatic conveyor.
            foreach (var item in _automationItems)
            {
                if (item.Completed || item.Model == _automationAttachedObject) continue;
                var candidatePosition = item.Model.Transform.Transform(new Point3D(0, 0, 0));
                var candidateDistance = (candidatePosition - fk.eePos).Length;
                if (candidateDistance < closestDistance)
                {
                    closestDistance = candidateDistance;
                    closestModel = item.Model;
                    closestItem = item;
                }
            }

            // Use the original demonstration object only when no automation object is available.
            if (closestModel == null && _pickableObject != null)
            {
                closestModel = _pickableObject;
                closestDistance = (_pickableObject.Transform.Transform(new Point3D(0, 0, 0)) - fk.eePos).Length;
            }

            if (closestModel == null)
            {
                if (ObjectStatusText != null) ObjectStatusText.Text = "Object: no available object";
                return;
            }

            var objectPosition = closestModel.Transform.Transform(new Point3D(0, 0, 0));
            var distance = objectPosition - fk.eePos;
            if (distance.Length > PickDistance)
            {
                if (ObjectStatusText != null) ObjectStatusText.Text = "Object: move it closer to gripper";
                return;
            }

            var inverse = toolOrientation;
            inverse.Invert();
            _objectToolOffset = inverse.Transform(distance);
            if (closestItem != null)
            {
                _automationAttachedObject = closestModel;
                _objectAttached = false;
            }
            else
            {
                _objectAttached = true;
            }
            if (ObjectStatusText != null) ObjectStatusText.Text = "Object: attached";
            ApplyFkToModels();
        }

        private void StartWristRotate_Click(object sender, RoutedEventArgs e)
        {
            _wristAutoTimer.Start();
        }

        private void StopWristRotate_Click(object sender, RoutedEventArgs e)
        {
            _wristAutoTimer.Stop();
        }
        // Activate the selected tool effect (cutting or welding) at the end effector position.
        private void ActivateTool_Click(object sender, RoutedEventArgs e)
        {
            _currentTool = (ToolMode)ToolModeCombo.SelectedIndex;
            // position effect at end effector
            var fk = ComputeForwardKinematics(_angles);
            var ee = fk.eePos;
            _toolEffectSphere.Transform = new TranslateTransform3D(ee.X, ee.Y, ee.Z + 0.02);
            // make color and start flashing
            if (_currentTool == ToolMode.Cut)
            {
                _toolEffectSphere.Material = CreateMaterial(Color.FromRgb(255, 80, 40));
            }
            else if (_currentTool == ToolMode.Weld)
            {
                _toolEffectSphere.Material = CreateMaterial(Color.FromRgb(255, 200, 0));
            }
            else
            {
                _toolEffectSphere.Material = CreateMaterial(Color.FromRgb(200, 200, 200));
            }
            _toolEffectTimer.Start();
        }
        // Deactivate the tool effect.
        private void AnimTimer_Tick(object sender, EventArgs e)
        {
            _animT += 0.03;
            // wave: oscillate elbow and wrist
            _angles[2] = 30 * Math.Sin(_animT * 2.0); // elbow
            _angles[3] = 20 * Math.Sin(_animT * 3.0 + 0.5); // wrist pitch
            Joint3Slider.Value = _angles[2];
            WristPitchSlider.Value = _angles[3];
            ApplyFkToModels();
        }
        // Handle mouse clicks in the 3D viewport to select a target point for IK.
        private HitTestResultBehavior ViewportHitTestResult(HitTestResult raw)
        {
            if (raw is RayHitTestResult rayResult)
            {
                var point = rayResult.PointHit;
                IkXText.Text = point.X.ToString("F2");
                IkYText.Text = point.Y.ToString("F2");
                IkZText.Text = point.Z.ToString("F2");
                if (_isDraggingTarget)
                {
                    // live solve
                    Solve6DTargetLive(point);
                }
                return HitTestResultBehavior.Stop;
            }
            return HitTestResultBehavior.Continue;
        }
        // Handle mouse left button down in the 3D viewport to initiate dragging for IK target selection.
        private void Viewport3D_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            var pos = e.GetPosition(MainViewport);
            bool ctrl = Keyboard.IsKeyDown(Key.LeftCtrl) || Keyboard.IsKeyDown(Key.RightCtrl);
            if (ctrl)
            {
                _isDraggingTarget = true;
                MainViewport.CaptureMouse();
            }
            VisualTreeHelper.HitTest(MainViewport, null,
                result => ViewportHitTestResult(result),
                new PointHitTestParameters(pos));
        }
        // Handle mouse move in the 3D viewport to update the IK target position while dragging.
        private void Viewport3D_MouseMove(object sender, MouseEventArgs e)
        {
            if (_isDraggingTarget && e.LeftButton == MouseButtonState.Pressed)
            {
                var pos = e.GetPosition(MainViewport);
                VisualTreeHelper.HitTest(MainViewport, null,
                    result => ViewportHitTestResult(result),
                    new PointHitTestParameters(pos));
            }
        }
        // Handle mouse left button up in the 3D viewport to stop dragging for IK target selection.
        private void Solve6DTargetLive(Point3D point)
        {
            // parse orientation from fields (if invalid use current)
            double rx = 0, ry = 0, rz = 0;
            double.TryParse(IkRxText.Text, out rx);
            double.TryParse(IkRyText.Text, out ry);
            double.TryParse(IkRzText.Text, out rz);
            var targetPos = point;
            Matrix3D Rz = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), rz);
            Matrix3D Ry = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 1, 0), ry);
            Matrix3D Rx = ForwardKinematics.RotateAxisAngle(new Vector3D(1, 0, 0), rx);
            var targetRot = Rx * Ry * Rz;
            bool useOri = UseOrientationCheck.IsChecked == true;
            // call solver (in-place)
            InverseKinematics6D.Solve6D(targetPos, targetRot, _angles, (angles) =>
            {
                var fk = ComputeForwardKinematics(angles);
                return (fk.eePos, fk.eeRot, fk.jointOrigins, fk.jointAxes);
            }, _minAngles, _maxAngles, useOri, 1e-3, 1e-2, 10, 0.08);
            // update UI (no messages)
            Joint1Slider.Value = _angles[0];
            Joint2Slider.Value = _angles[1];
            Joint3Slider.Value = _angles[2];
            WristPitchSlider.Value = _angles[3];
            ApplyFkToModels();
        }

        // Keyframe/trajectory handlers
        private void RecordKeyframe_Click(object sender, RoutedEventArgs e)
        {
            if (_angles == null) return;
            var copy = new double[_angles.Length];
            Array.Copy(_angles, copy, _angles.Length);
            double time = 0;
            if (_keyframes.Count == 0) time = 0; else time = _keyframes[_keyframes.Count - 1].Time + 1.0;
            _keyframes.Add(new Keyframe { Time = time, Angles = copy });
            MessageBox.Show($"Recorded keyframe #{_keyframes.Count} at t={time:F2}s");
        }
        // Clear all recorded keyframes
        private void ClearKeyframes_Click(object sender, RoutedEventArgs e)
        {
            _keyframes.Clear();
            MessageBox.Show("Keyframes cleared");
        }
        // Play the recorded trajectory using cubic Hermite interpolation
        private void PlayTrajectory_Click(object sender, RoutedEventArgs e)
        {
            if (_keyframes.Count < 2) { MessageBox.Show("Need at least 2 keyframes to play."); return; }
            // ensure keyframes sorted by time
            _keyframes.Sort((a, b) => a.Time.CompareTo(b.Time));
            _trajIndex = 0;
            _trajSegT = 0;
            _trajTime = 0;
            _trajTimer.Start();
        }
        // Stop the trajectory playback
        private void StopTrajectory_Click(object sender, RoutedEventArgs e)
        {
            _trajTimer.Stop();
        }
        // Timer tick handler for trajectory playback, interpolating joint angles based on keyframes
        private void TrajTimer_Tick(object sender, EventArgs e)
        {
            if (_keyframes.Count < 2) { _trajTimer.Stop(); return; }
            double dt = _trajTimer.Interval.TotalSeconds;
            _trajTime += dt;
            double total = _keyframes[_keyframes.Count - 1].Time;
            if (_trajTime > total) { _trajTimer.Stop(); return; }

            // find segment idx where k[i].Time <= t < k[i+1].Time
            int idx = 0;
            for (int i = 0; i < _keyframes.Count - 1; i++)
            {
                if (_trajTime >= _keyframes[i].Time && _trajTime <= _keyframes[i + 1].Time) { idx = i; break; }
                if (_trajTime > _keyframes[i + 1].Time) continue;
            }
            int i1 = idx;
            int i0 = Math.Max(0, i1 - 1);
            int i2 = Math.Min(_keyframes.Count - 1, i1 + 1);
            int i3 = Math.Min(_keyframes.Count - 1, i1 + 2);

            var k0 = _keyframes[i0];
            var k1 = _keyframes[i1];
            var k2 = _keyframes[i2];
            var k3 = _keyframes[i3];

            double t0 = k1.Time;
            double t1 = k2.Time;
            double segDur = Math.Max(1e-6, t1 - t0);
            double s = (_trajTime - t0) / segDur;
            s = Math.Max(0.0, Math.Min(1.0, s));

            for (int j = 0; j < _angles.Length; j++)
            {
                double p0 = k0.Angles[j];
                double p1 = k1.Angles[j];
                double p2 = k2.Angles[j];
                double p3 = k3.Angles[j];

                // compute tangents (time-aware)
                double denom1 = Math.Max(1e-6, k2.Time - k0.Time);
                double m1 = (p2 - p0) / denom1;
                double denom2 = Math.Max(1e-6, k3.Time - k1.Time);
                double m2 = (p3 - p1) / denom2;

                double t2 = s * s; double t3 = t2 * s;
                double h00 = 2 * t3 - 3 * t2 + 1;
                double h10 = t3 - 2 * t2 + s;
                double h01 = -2 * t3 + 3 * t2;
                double h11 = t3 - t2;

                double value = h00 * p1 + h10 * (m1 * segDur) + h01 * p2 + h11 * (m2 * segDur);
                _angles[j] = value;
            }

            // update sliders and FK
            Joint1Slider.Value = _angles[0];
            Joint2Slider.Value = _angles[1];
            Joint3Slider.Value = _angles[2];
            WristPitchSlider.Value = _angles[3];
            ApplyFkToModels();
        }
        // Export the recorded keyframes to a JSON file
        private void ExportKeyframes_Click(object sender, RoutedEventArgs e)
        {
            if (_keyframes.Count == 0) { MessageBox.Show("No keyframes to export."); return; }
            var dlg = new SaveFileDialog() { Filter = "JSON files (*.json)|*.json|All files|*.*", FileName = "keyframes.json" };
            if (dlg.ShowDialog() == true)
            {
                var options = new JsonSerializerOptions { WriteIndented = true };
                System.IO.File.WriteAllText(dlg.FileName, JsonSerializer.Serialize(_keyframes, options));
                MessageBox.Show("Exported keyframes.");
            }
        }
        // Import keyframes from a JSON file
        private void ImportKeyframes_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog() { Filter = "JSON files (*.json)|*.json|All files|*.*" };
            if (dlg.ShowDialog() == true)
            {
                try
                {
                    var json = System.IO.File.ReadAllText(dlg.FileName);
                    var imported = JsonSerializer.Deserialize<System.Collections.Generic.List<Keyframe>>(json);
                    if (imported != null)
                    {
                        _keyframes = imported;
                        MessageBox.Show($"Imported {_keyframes.Count} keyframes.");
                    }
                }
                catch (Exception ex)
                {
                    MessageBox.Show("Import failed: " + ex.Message);
                }
            }
        }
    }
}
