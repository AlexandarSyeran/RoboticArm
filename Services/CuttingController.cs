using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    public enum CuttingPhase
    {
        Idle,
        Approach,
        Descend,
        Cutting,
        Retract,
        Complete
    }

    // Drives the laser cutting scenario. For every object the arm moves above
    // the entry side, lowers the tool onto the cut line, sweeps the beam across
    // the object and only then splits it, so the cut is a real stroke instead
    // of a timer at a fixed pose.
    public sealed class CuttingController
    {
        private const double HoverClearance = 0.45;
        private const double StrokeMargin = 0.26;
        private const double ContactGap = 0.02;
        private const double BeamStandoff = 0.12;
        private const double CutSpeedScale = 0.65;
        private const double PathPoseToleranceDegrees = 2.5;
        private const double GoalPoseToleranceDegrees = 0.35;
        private const double GoalPositionTolerance = 0.04;
        private const int StrokeSamples = 8;

        private readonly RobotArmModel _robot;
        private readonly IReadOnlyList<AutomationItem> _items;
        private readonly JointMotion _motion;
        // Snapshot taken at Start: the live list changes as objects are split
        // into pieces, so the run must keep its own ordering.
        private List<AutomationItem> _targets = new();

        private readonly List<double[]> _strokePlan = new();
        private double[]? _hoverGoal;
        private double[]? _startGoal;
        private int _strokeIndex;
        private double _contactY;
        private double _strokeStartX;
        private double _strokeEndX;
        private Vector3D _cutNormal = new(0, -1, 0);
        private int _index;

        public CuttingController(RobotArmModel robot, IReadOnlyList<AutomationItem> items)
        {
            _robot = robot;
            _items = items;
            _motion = new JointMotion(robot.Angles.Length);
        }

        public double SpeedMultiplier { get; set; } = SceneConfig.DefaultSpeedMultiplier;
        public double TickSeconds { get; set; } = SceneConfig.TimerMilliseconds / 1000.0;

        public CuttingPhase Phase { get; private set; } = CuttingPhase.Idle;
        public string Status { get; private set; } = "Cutting: idle";
        public AutomationItem? Target { get; private set; }
        public Point3D CutPoint { get; private set; }
        public double Progress { get; private set; }
        public int ObjectsCut { get; private set; }
        public int TargetCount => _targets.Count;

        public bool IsRunning => Phase != CuttingPhase.Idle && Phase != CuttingPhase.Complete;
        public bool IsBeamOn => Phase == CuttingPhase.Cutting;

        public event Action? StatusChanged;
        public event Action? Finished;
        public event Action<AutomationItem, Vector3D>? ItemCut;

        public bool Start()
        {
            _targets = _items.Where(item => !item.Completed).ToList();
            if (_targets.Count == 0)
            {
                SetStatus("Cutting: no objects to cut");
                return false;
            }

            _index = 0;
            ObjectsCut = 0;
            _motion.Reset();
            return BeginItem();
        }

        public void Stop(string reason)
        {
            _motion.Reset();
            Phase = CuttingPhase.Idle;
            Target = null;
            Progress = 0;
            _strokePlan.Clear();
            _hoverGoal = null;
            _startGoal = null;
            SetStatus($"Cutting: stopped ({reason})");
        }

        public void ResetState()
        {
            _motion.Reset();
            Phase = CuttingPhase.Idle;
            Target = null;
            Progress = 0;
            ObjectsCut = 0;
            _targets.Clear();
            _strokePlan.Clear();
            _hoverGoal = null;
            _startGoal = null;
            _index = 0;
            _strokeIndex = 0;
            _cutNormal = new Vector3D(0, -1, 0);
            SetStatus("Cutting: idle");
        }

        public void Tick()
        {
            if (!IsRunning)
                return;

            switch (Phase)
            {
                case CuttingPhase.Approach:
                    SetStatus($"Cutting {_index + 1}/{_targets.Count}: moving above object");
                    if (StepTo(_hoverGoal!, 1.0) <= GoalPoseToleranceDegrees + 0.15)
                        Transition(CuttingPhase.Descend, $"Cutting {_index + 1}/{_targets.Count}: lowering the tool");
                    break;

                case CuttingPhase.Descend:
                    SetStatus($"Cutting {_index + 1}/{_targets.Count}: lowering the tool");
                    if (StepTo(_startGoal!, 1.0) <= GoalPoseToleranceDegrees)
                        Transition(CuttingPhase.Cutting, $"Cutting {_index + 1}/{_targets.Count}: sweeping the cut");
                    break;

                case CuttingPhase.Cutting:
                    SetStatus($"Cutting {_index + 1}/{_targets.Count}: sweeping the cut");
                    if (_strokeIndex >= _strokePlan.Count)
                        break;

                    double remaining = StepTo(_strokePlan[_strokeIndex], CutSpeedScale);
                    UpdateCutPoint();
                    Progress = Math.Clamp(
                        (_strokeEndX - _strokeStartX) == 0
                            ? 1.0
                            : (CutPoint.X - _strokeStartX) / (_strokeEndX - _strokeStartX),
                        0,
                        1);

                    bool last = _strokeIndex == _strokePlan.Count - 1;
                    if (remaining <= (last ? GoalPoseToleranceDegrees : PathPoseToleranceDegrees))
                    {
                        if (!last)
                        {
                            _strokeIndex++;
                            break;
                        }

                        Progress = 1.0;
                        ObjectsCut++;
                        ItemCut?.Invoke(Target!, _cutNormal);
                        Transition(CuttingPhase.Retract, $"Cutting {_index + 1}/{_targets.Count}: object cut");
                    }
                    break;

                case CuttingPhase.Retract:
                    SetStatus($"Cutting {_index + 1}/{_targets.Count}: retracting");
                    if (StepTo(_hoverGoal!, 1.0) <= GoalPoseToleranceDegrees + 0.15)
                    {
                        _index++;
                        if (_index >= _targets.Count)
                            CompleteFlow();
                        else
                            BeginItem();
                    }
                    break;
            }
        }

        // Beam origin: the tool tip in world space for the current pose.
        public Point3D ToolTip()
        {
            var fk = _robot.ForwardKinematics();
            return ToolTip(fk, _robot.ToolOrientation(fk));
        }

        private bool BeginItem()
        {
            Target = _targets[_index];
            Progress = 0;
            _strokeIndex = 0;
            _strokePlan.Clear();
            _hoverGoal = null;
            _startGoal = null;

            var half = Target.HalfSize;
            double topY = Target.Position.Y + half.Y;
            _contactY = topY + ContactGap;
            double eeY = topY + SceneConfig.CutToolTipLength + BeamStandoff;
            double z = Target.Position.Z;
            _strokeStartX = Target.Position.X - half.X - StrokeMargin;
            _strokeEndX = Target.Position.X + half.X + StrokeMargin;

            var start = new Point3D(_strokeStartX, eeY, z);
            var end = new Point3D(_strokeEndX, eeY, z);
            var hover = new Point3D(
                _strokeStartX,
                Math.Max(eeY + HoverClearance, Target.Position.Y + SceneConfig.CutHoverHeight),
                z);

            var down = new Vector3D(0, -1, 0);
            if (!PlanFromCurrent(hover, down, out _hoverGoal))
            {
                Fail("the arm cannot reach a position above the object");
                return false;
            }

            if (!TryPlan(_hoverGoal, start, down, out _startGoal))
            {
                Fail("the arm cannot reach the cut line");
                return false;
            }

            // Follow the direction the arm actually holds at the entry point;
            // it is nearly vertical, so the split plane stays horizontal.
            _cutNormal = ToolDirectionFor(_startGoal);

            double[] seed = _startGoal;
            for (int sample = 1; sample <= StrokeSamples; sample++)
            {
                double x = start.X + (end.X - start.X) * sample / StrokeSamples;
                var point = new Point3D(x, eeY, z);
                if (!TryPlan(seed, point, _cutNormal, out var goal))
                {
                    Fail("the arm cannot follow the cut line");
                    return false;
                }

                _strokePlan.Add(goal);
                seed = goal;
            }

            CutPoint = new Point3D(Target.Position.X, _contactY, z);
            Transition(CuttingPhase.Approach, $"Cutting {_index + 1}/{_targets.Count}: moving above object");
            return true;
        }

        private double StepTo(double[] goal, double speedScale)
        {
            return _motion.Step(_robot, goal, SpeedMultiplier * speedScale, TickSeconds);
        }

        // The beam contact stays on the object's top surface while the tip
        // sweeps along the cut line.
        private void UpdateCutPoint()
        {
            var tip = ToolTip();
            CutPoint = new Point3D(tip.X, _contactY, tip.Z);
        }

        private bool PlanFromCurrent(Point3D target, Vector3D? direction, out double[] goal)
        {
            if (TryPlan(_robot.Angles, target, direction, out goal))
                return true;

            return TryPlan(_robot.InitialFoldedAngles, target, direction, out goal);
        }

        private bool TryPlan(double[] seed, Point3D target, Vector3D? direction, out double[] goal)
        {
            var twin = NewTwin();
            Array.Copy(seed, twin.Angles, seed.Length);

            bool solved = twin.SolveGoal(target, direction);
            if (!solved && direction != null)
                solved = twin.SolveGoal(target, null);

            goal = (double[])twin.Angles.Clone();
            return solved && (twin.ForwardKinematics().EePos - target).Length <= GoalPositionTolerance;
        }

        private Vector3D ToolDirectionFor(double[] angles)
        {
            var twin = NewTwin();
            Array.Copy(angles, twin.Angles, angles.Length);
            var direction = twin.ToolDirection();
            return direction;
        }

        private RobotArmModel NewTwin() => new()
        {
            L1 = _robot.L1,
            L2 = _robot.L2,
            L3 = _robot.L3,
            WristRoll = _robot.WristRoll
        };

        private void Transition(CuttingPhase phase, string status)
        {
            Phase = phase;
            SetStatus(status);
        }

        private void CompleteFlow()
        {
            Target = null;
            Progress = 1;
            Phase = CuttingPhase.Complete;
            SetStatus($"Cutting: finished, {ObjectsCut} of {_targets.Count} objects cut");
            Finished?.Invoke();
        }

        private void Fail(string reason)
        {
            Phase = CuttingPhase.Complete;
            Target = null;
            Progress = 0;
            SetStatus($"Cutting: stopped ({reason})");
            Finished?.Invoke();
        }

        private void SetStatus(string status)
        {
            if (Status == status)
                return;

            Status = status;
            StatusChanged?.Invoke();
        }

        private Point3D ToolTip(FkResult fk, Matrix3D orientation)
        {
            var forward = orientation.Transform(new Vector3D(1, 0, 0));
            forward.Normalize();
            return new Point3D(
                fk.EePos.X + forward.X * SceneConfig.CutToolTipLength,
                fk.EePos.Y + forward.Y * SceneConfig.CutToolTipLength,
                fk.EePos.Z + forward.Z * SceneConfig.CutToolTipLength);
        }
    }
}
