using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Threading;
using RoboticArm.Models;
using RoboticArm.Services;
using RoboticArm.ViewModels;

namespace RoboticArm.Rendering
{
    public sealed class RobotSceneRenderer : IDisposable
    {
        private const int GripperFingerCount = 4;

        private readonly Viewport3D _viewport;
        private readonly MainViewModel _vm;

        private Model3DGroup? _sceneGroup;
        private GeometryModel3D?[] _linkModels = Array.Empty<GeometryModel3D?>();
        private MatrixTransform3D[] _linkTransforms = Array.Empty<MatrixTransform3D>();
        private GeometryModel3D? _eeSphereModel;
        private GeometryModel3D[] _gripperFingers = Array.Empty<GeometryModel3D>();
        private GeometryModel3D? _toolEffectSphere;
        private GeometryModel3D[] _wristAxes = Array.Empty<GeometryModel3D>();
        private GeometryModel3D? _radarModel;
        private GeometryModel3D? _cutBeam;
        private GeometryModel3D? _cutSpark;

        private readonly Dictionary<AutomationItem, GeometryModel3D> _itemModels = new();
        private readonly Dictionary<Obstacle, GeometryModel3D> _obstacleModels = new();
        private readonly Dictionary<Obstacle, GeometryModel3D> _radarHits = new();

        private static readonly Transform3D HiddenTransform = new TranslateTransform3D(0, -100, 0);

        private DispatcherTimer? _toolEffectTimer;
        private int _toolFlash;
        private ToolMode _toolMode = ToolMode.Grip;
        private bool _toolEffectActive;
        private bool _built;

        private static readonly Color[] ItemPalette =
        {
            Color.FromRgb(235, 80, 70),
            Color.FromRgb(70, 150, 235),
            Color.FromRgb(240, 190, 55),
            Color.FromRgb(170, 90, 220)
        };

        public RobotSceneRenderer(Viewport3D viewport, MainViewModel viewModel)
        {
            _viewport = viewport;
            _vm = viewModel;

            _vm.PoseChanged += OnPoseChanged;
            _vm.AutomationSceneChanged += OnAutomationSceneChanged;
            _vm.WristAxesVisibilityChanged += SetWristAxesVisible;
            _vm.ToolActivated += FlashTool;
        }

        public void Build()
        {
            if (_built)
                return;

            _built = true;
            _sceneGroup = new Model3DGroup();
            _sceneGroup.Children.Add(new AmbientLight(Color.FromRgb(60, 60, 60)));
            _sceneGroup.Children.Add(new DirectionalLight(Colors.White, new Vector3D(-1, -1, -2)));

            var gridModel = new GeometryModel3D(
                MeshBuilder.CreateGrid(10, 0.5),
                new DiffuseMaterial(new SolidColorBrush(Color.FromRgb(40, 40, 40))));
            _sceneGroup.Children.Add(gridModel);

            AddBase();
            AddRobot();
            AddGripper();
            AddToolEffect();
            AddCutTool();
            AddWristAxes();
            AddRadar();
            AddConveyorAndZones();

            RebuildAutomationItems();
            RebuildObstacles();

            _viewport.Children.Add(new ModelVisual3D { Content = _sceneGroup });
            UpdateAll();
        }

        public void Dispose()
        {
            _vm.PoseChanged -= OnPoseChanged;
            _vm.AutomationSceneChanged -= OnAutomationSceneChanged;
            _vm.WristAxesVisibilityChanged -= SetWristAxesVisible;
            _vm.ToolActivated -= FlashTool;
            _toolEffectTimer?.Stop();
        }

        private void AddBase()
        {
            var baseModel = new GeometryModel3D(
                MeshBuilder.CreateBox(1.6, 0.3, 1.6),
                CreateMaterial(Color.FromRgb(120, 120, 120)));
            baseModel.Transform = new TranslateTransform3D(0, 0.15, 0);
            _sceneGroup!.Children.Add(baseModel);
        }

        private void AddRobot()
        {
            int count = _vm.Robot.Angles.Length;
            _linkModels = new GeometryModel3D?[count];
            _linkTransforms = new MatrixTransform3D[count];

            double thickness = 0.28;
            var colors = new[]
            {
                Color.FromRgb(120, 120, 120),
                Color.FromRgb(200, 80, 80),
                Color.FromRgb(80, 160, 200),
                Color.FromRgb(160, 200, 90),
                Color.FromRgb(200, 160, 220)
            };
            var radii = new[] { 0.0, thickness / 2.0, thickness * 0.45, thickness * 0.4, thickness * 0.35 };
            var segments = new[] { 0, 40, 36, 28, 20 };

            for (int i = 0; i < count; i++)
            {
                var transform = new MatrixTransform3D(Matrix3D.Identity);
                _linkTransforms[i] = transform;

                double length = _vm.Robot.LinkLengths[i];
                if (length > 0.01)
                {
                    var mesh = MeshBuilder.CreateCylinder(length, radii[i], segments[i]);
                    var model = new GeometryModel3D(mesh, CreateMaterial(colors[i])) { Transform = transform };
                    _linkModels[i] = model;
                    _sceneGroup!.Children.Add(model);
                }

                _sceneGroup!.Children.Add(new GeometryModel3D(
                    MeshBuilder.CreateSphere(0.12, 12, 8),
                    CreateMaterial(Color.FromRgb(220, 220, 220))));
            }

            _eeSphereModel = new GeometryModel3D(
                MeshBuilder.CreateSphere(0.14, 12, 8),
                CreateMaterial(Color.FromRgb(255, 200, 50)));
            _sceneGroup.Children.Add(_eeSphereModel);
        }

        private void AddGripper()
        {
            _gripperFingers = new GeometryModel3D[GripperFingerCount];
            for (int i = 0; i < GripperFingerCount; i++)
            {
                var finger = new GeometryModel3D(
                    MeshBuilder.CreateBox(0.4, 0.06, 0.12),
                    CreateMaterial(Color.FromRgb(200, 200, 200)))
                {
                    Transform = new MatrixTransform3D(Matrix3D.Identity)
                };

                _gripperFingers[i] = finger;
                _sceneGroup!.Children.Add(finger);
            }
        }

        private void AddToolEffect()
        {
            _toolEffectSphere = new GeometryModel3D(
                MeshBuilder.CreateSphere(0.08, 12, 8),
                CreateMaterial(Color.FromRgb(255, 50, 50)))
            {
                Transform = new TranslateTransform3D(0, -100, 0)
            };

            _sceneGroup!.Children.Add(_toolEffectSphere);
        }

        private void AddCutTool()
        {
            var laserColor = Color.FromRgb(255, 70, 30);
            _cutBeam = new GeometryModel3D(
                MeshBuilder.CreateCylinder(1.0, 0.022, 12),
                CreateBeamMaterial(laserColor))
            {
                Transform = HiddenTransform
            };

            _cutSpark = new GeometryModel3D(
                MeshBuilder.CreateSphere(0.05, 10, 8),
                CreateBeamMaterial(Color.FromRgb(255, 220, 90)))
            {
                Transform = HiddenTransform
            };

            _sceneGroup!.Children.Add(_cutBeam);
            _sceneGroup.Children.Add(_cutSpark);
        }

        private void AddWristAxes()
        {
            _wristAxes = new[]
            {
                CreateAxisModel(Colors.Red),
                CreateAxisModel(Colors.Green),
                CreateAxisModel(Colors.Blue)
            };

            foreach (var axis in _wristAxes)
            {
                if (_vm.ShowWristAxes)
                    _sceneGroup!.Children.Add(axis);
            }
        }

        private void AddRadar()
        {
            _radarModel = new GeometryModel3D(
                MeshBuilder.CreateCylinder(0.18, 0.025, 20),
                CreateTransparentMaterial(Color.FromArgb(150, 40, 220, 255)));
            _sceneGroup!.Children.Add(_radarModel);
        }

        private void AddConveyorAndZones()
        {
            var conveyor = new GeometryModel3D(
                MeshBuilder.CreateBox(8.4, 0.12, 2.4),
                CreateTransparentMaterial(Color.FromArgb(105, 125, 195, 220)))
            {
                Transform = new TranslateTransform3D(0, 0.06, 0)
            };
            _sceneGroup!.Children.Add(conveyor);

            var pickupZone = new GeometryModel3D(
                MeshBuilder.CreateBox(0.9, 0.025, 0.9),
                CreateMaterial(Color.FromRgb(50, 125, 210)))
            {
                Transform = new TranslateTransform3D(_vm.PickupZone.X, 0.13, _vm.PickupZone.Z)
            };
            _sceneGroup.Children.Add(pickupZone);

            var dropZone = new GeometryModel3D(
                MeshBuilder.CreateBox(1.0, 0.025, 1.0),
                CreateMaterial(Color.FromRgb(40, 170, 90)))
            {
                Transform = new TranslateTransform3D(_vm.DropTarget.X, 0.13, _vm.DropTarget.Z)
            };
            _sceneGroup.Children.Add(dropZone);
        }

        private void RebuildAutomationItems()
        {
            if (_sceneGroup == null)
                return;

            foreach (var model in _itemModels.Values)
                _sceneGroup.Children.Remove(model);

            _itemModels.Clear();

            foreach (var item in _vm.Items)
            {
                var color = ItemPalette[Math.Abs(item.PaletteIndex) % ItemPalette.Length];
                if (item.CutNormal != null && item.CutSide != 0)
                    color = Blend(color, Colors.Black, 0.35);

                var model = new GeometryModel3D(CreateItemMesh(item), CreateMaterial(color))
                {
                    BackMaterial = CreateMaterial(color),
                    Transform = new TranslateTransform3D(item.Position.X, item.Position.Y, item.Position.Z)
                };

                _itemModels[item] = model;
                _sceneGroup.Children.Add(model);
            }
        }

        // Whole blocks are drawn from their half extents; cut pieces are rebuilt
        // from the original block and the plane stored on the item.
        private static MeshGeometry3D CreateItemMesh(AutomationItem item)
        {
            if (item.CutNormal == null)
            {
                return MeshBuilder.CreateBox(
                    item.HalfSize.X * 2,
                    item.HalfSize.Y * 2,
                    item.HalfSize.Z * 2);
            }

            var (keep, offcut) = BoxSplitter.SplitPieces(
                item.CutCenter, CutPlanner.CutHalfSize, item.CutNormal.Value);
            return item.CutSide == 0 ? keep.Mesh : offcut.Mesh;
        }

        private void RebuildObstacles()
        {
            if (_sceneGroup == null)
                return;

            foreach (var model in _obstacleModels.Values)
                _sceneGroup.Children.Remove(model);
            foreach (var hit in _radarHits.Values)
                _sceneGroup.Children.Remove(hit);

            _obstacleModels.Clear();
            _radarHits.Clear();

            foreach (var obstacle in _vm.Obstacles)
            {
                var model = new GeometryModel3D(
                    MeshBuilder.CreateBox(
                        obstacle.HalfSize.X * 2,
                        obstacle.HalfSize.Y * 2,
                        obstacle.HalfSize.Z * 2),
                    CreateMaterial(Color.FromRgb(180, 75, 70)))
                {
                    Transform = new TranslateTransform3D(
                        obstacle.Center.X, obstacle.Center.Y, obstacle.Center.Z)
                };

                var hit = new GeometryModel3D(
                    MeshBuilder.CreateSphere(0.07, 8, 6),
                    CreateMaterial(Color.FromRgb(255, 215, 40)))
                {
                    Transform = RadarHitTransform(obstacle)
                };

                _obstacleModels[obstacle] = model;
                _radarHits[obstacle] = hit;
                _sceneGroup.Children.Add(model);
                _sceneGroup.Children.Add(hit);
            }
        }

        private void OnPoseChanged() => UpdateAll();

        private void OnAutomationSceneChanged()
        {
            if (!_built)
                return;

            RebuildAutomationItems();
            RebuildObstacles();
            UpdateAll();
        }

        private void UpdateAll()
        {
            if (!_built || _sceneGroup == null)
                return;

            UpdateRobotPose();
            UpdateItemPositions();
            UpdateRadarHits();
            UpdateCutTool();
        }

        private void UpdateRobotPose()
        {
            var robot = _vm.Robot;
            var fk = robot.ForwardKinematics();
            var eePos = fk.EePos;

            Matrix3D m = Kinematics.ForwardKinematics.Translate(0, RobotArmModel.BaseHeight / 2.0, 0);
            var localAxes = RobotArmModel.JointAxesLocal;

            for (int i = 0; i < _linkTransforms.Length; i++)
            {
                var rotation = Kinematics.ForwardKinematics.RotateAxisAngle(localAxes[i], robot.Angles[i]);
                m = rotation * m;

                double length = robot.LinkLengths[i];
                var center = Kinematics.ForwardKinematics.Translate(length / 2.0, 0, 0) * m;
                var align = Kinematics.ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), -90);
                _linkTransforms[i].Matrix = align * center;

                m = Kinematics.ForwardKinematics.Translate(length, 0, 0) * m;
            }

            _eeSphereModel!.Transform = new TranslateTransform3D(eePos.X, eePos.Y, eePos.Z);

            var wristRoll = Kinematics.ForwardKinematics.RotateAxisAngle(
                new Vector3D(1, 0, 0), robot.WristRoll);
            var toolOrientation = fk.EeRot * wristRoll;
            UpdateWristAxes(eePos, toolOrientation);
            UpdateGripper(eePos, toolOrientation, robot);

            if (_toolEffectActive)
            {
                _toolEffectSphere!.Transform = new TranslateTransform3D(eePos.X, eePos.Y, eePos.Z + 0.02);
            }

            if (_radarModel != null)
            {
                _radarModel.Transform = new TranslateTransform3D(eePos.X, eePos.Y, eePos.Z);
            }
        }

        private void UpdateGripper(Point3D eePos, Matrix3D toolOrientation, RobotArmModel robot)
        {
            double fingerLength = 0.4;
            double fingerRadius = 0.12 + 0.06 * robot.GripperOpen;

            for (int i = 0; i < _gripperFingers.Length; i++)
            {
                double angle = i * Math.PI / 2.0;
                var centerLocal = new Vector3D(
                    fingerLength / 2.0 + 0.06,
                    Math.Cos(angle) * fingerRadius,
                    Math.Sin(angle) * fingerRadius);

                var centerOffset = toolOrientation.Transform(centerLocal);
                var centerWorld = new Point3D(
                    eePos.X + centerOffset.X,
                    eePos.Y + centerOffset.Y,
                    eePos.Z + centerOffset.Z);

                var group = new Transform3DGroup();
                group.Children.Add(new RotateTransform3D(
                    new QuaternionRotation3D(MatrixToQuaternion(toolOrientation))));
                group.Children.Add(new TranslateTransform3D(centerWorld.X, centerWorld.Y, centerWorld.Z));
                _gripperFingers[i].Transform = group;
            }
        }

        private void UpdateWristAxes(Point3D origin, Matrix3D orientation)
        {
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

                var group = new Transform3DGroup();
                group.Children.Add(new RotateTransform3D(
                    new QuaternionRotation3D(RotationFromTo(new Vector3D(0, 1, 0), direction))));
                group.Children.Add(new TranslateTransform3D(midpoint.X, midpoint.Y, midpoint.Z));
                _wristAxes[i].Transform = group;
            }
        }

        private void UpdateItemPositions()
        {
            foreach (var pair in _itemModels)
            {
                var position = pair.Key.Position;
                pair.Value.Transform = new TranslateTransform3D(position.X, position.Y, position.Z);
            }
        }

        private void UpdateRadarHits()
        {
            foreach (var pair in _radarHits)
            {
                pair.Value.Transform = RadarHitTransform(pair.Key);
            }
        }

        private static TranslateTransform3D RadarHitTransform(Obstacle obstacle)
        {
            return new TranslateTransform3D(
                obstacle.Center.X,
                obstacle.Center.Y + obstacle.HalfSize.Y + 0.08,
                obstacle.Center.Z);
        }

        private void UpdateCutTool()
        {
            if (_cutBeam == null || _cutSpark == null)
                return;

            if (!_vm.IsCutBeamOn)
            {
                _cutBeam.Transform = HiddenTransform;
                _cutSpark.Transform = HiddenTransform;
                return;
            }

            var robot = _vm.Robot;
            var fk = robot.ForwardKinematics();
            var orientation = robot.ToolOrientation(fk);
            var forward = orientation.Transform(new Vector3D(1, 0, 0));
            forward.Normalize();

            var tip = new Point3D(
                fk.EePos.X + forward.X * 0.15,
                fk.EePos.Y + forward.Y * 0.15,
                fk.EePos.Z + forward.Z * 0.15);
            var target = _vm.CutPoint;
            var delta = target - tip;
            double length = delta.Length;

            if (length < 0.02)
            {
                _cutBeam.Transform = HiddenTransform;
                _cutSpark.Transform = HiddenTransform;
                return;
            }

            var direction = delta;
            direction.Normalize();
            var midpoint = tip + delta * 0.5;

            var group = new Transform3DGroup();
            group.Children.Add(new ScaleTransform3D(1, length, 1));
            group.Children.Add(new RotateTransform3D(
                new QuaternionRotation3D(RotationFromTo(new Vector3D(0, 1, 0), direction))));
            group.Children.Add(new TranslateTransform3D(midpoint.X, midpoint.Y, midpoint.Z));

            _cutBeam.Transform = group;
            _cutSpark.Transform = new TranslateTransform3D(target.X, target.Y, target.Z);
        }

        private static Color Blend(Color from, Color to, double amount)
        {
            return Color.FromRgb(
                (byte)(from.R + (to.R - from.R) * amount),
                (byte)(from.G + (to.G - from.G) * amount),
                (byte)(from.B + (to.B - from.B) * amount));
        }

        private static Material CreateBeamMaterial(Color color)
        {
            var material = new MaterialGroup();
            material.Children.Add(new DiffuseMaterial(new SolidColorBrush(color)));
            material.Children.Add(new EmissiveMaterial(new SolidColorBrush(color)));
            return material;
        }

        private void SetWristAxesVisible(bool visible)
        {
            if (_sceneGroup == null)
                return;

            foreach (var axis in _wristAxes)
            {
                bool contains = _sceneGroup.Children.Contains(axis);
                if (visible && !contains)
                    _sceneGroup.Children.Add(axis);
                else if (!visible && contains)
                    _sceneGroup.Children.Remove(axis);
            }
        }

        private void FlashTool(ToolMode mode)
        {
            _toolMode = mode;
            _toolEffectActive = true;
            _toolFlash = 0;

            var ee = _vm.Robot.ForwardKinematics().EePos;
            _toolEffectSphere!.Transform = new TranslateTransform3D(ee.X, ee.Y, ee.Z + 0.02);

            _toolEffectSphere.Material = mode switch
            {
                ToolMode.Cut => CreateMaterial(Color.FromRgb(255, 80, 40)),
                ToolMode.Weld => CreateMaterial(Color.FromRgb(255, 200, 0)),
                _ => CreateMaterial(Color.FromRgb(200, 200, 200))
            };

            _toolEffectTimer ??= CreateToolEffectTimer();
            _toolEffectTimer.Start();
        }

        private DispatcherTimer CreateToolEffectTimer()
        {
            var timer = new DispatcherTimer { Interval = TimeSpan.FromMilliseconds(100) };
            timer.Tick += (_, _) =>
            {
                _toolFlash++;

                if (_toolEffectSphere?.Material is MaterialGroup group &&
                    group.Children.Count > 0 &&
                    group.Children[0] is DiffuseMaterial diffuse)
                {
                    if (_toolMode == ToolMode.Cut)
                    {
                        diffuse.Brush = new SolidColorBrush(Color.FromRgb(
                            255, (byte)(50 + (_toolFlash % 2) * 100), 50));
                    }
                    else if (_toolMode == ToolMode.Weld)
                    {
                        diffuse.Brush = new SolidColorBrush(Color.FromRgb(
                            255, (byte)(150 + (_toolFlash % 2) * 50), 0));
                    }
                }

                if (_toolFlash > 8)
                {
                    timer.Stop();
                    _toolEffectActive = false;
                    _toolEffectSphere!.Transform = new TranslateTransform3D(0, -100, 0);
                }
            };

            return timer;
        }

        private static Material CreateMaterial(Color color)
        {
            var material = new MaterialGroup();
            material.Children.Add(new DiffuseMaterial(new SolidColorBrush(color)));
            material.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.White), 32));
            return material;
        }

        private static Material CreateTransparentMaterial(Color color)
        {
            var brush = new SolidColorBrush(color) { Opacity = color.A / 255.0 };
            var material = new MaterialGroup();
            material.Children.Add(new DiffuseMaterial(brush));
            material.Children.Add(new SpecularMaterial(new SolidColorBrush(Colors.White), 96));
            return material;
        }

        private static GeometryModel3D CreateAxisModel(Color color)
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
                var perpendicular = Vector3D.CrossProduct(from, new Vector3D(1, 0, 0));
                if (perpendicular.Length < 1e-6)
                    perpendicular = Vector3D.CrossProduct(from, new Vector3D(0, 1, 0));

                perpendicular.Normalize();
                return new Quaternion(perpendicular, 180);
            }

            var axis = Vector3D.CrossProduct(from, to);
            axis.Normalize();
            return new Quaternion(axis, Math.Acos(dot) * 180.0 / Math.PI);
        }

        private static Quaternion MatrixToQuaternion(Matrix3D m)
        {
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
            else if (m.M11 > m.M22 && m.M11 > m.M33)
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

            var q = new Quaternion(x, y, z, w);
            q.Normalize();
            return q;
        }
    }
}
