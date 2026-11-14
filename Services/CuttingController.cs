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

    // Drives the laser cutting scenario: the arm visits every object on the
    // table, lowers the tool above it and burns a cut whose plane follows the
    // tool orientation at that moment.
    public sealed class CuttingController
    {
        private const int StuckTickLimit = 240;
        private const double HoverHeight = 0.75;
        private const double CutHeight = 0.55;
        private const double ToolTipLength = 0.15;
        private const double ProgressEpsilon = 1e-5;

        private readonly RobotArmModel _robot;
        private readonly IReadOnlyList<AutomationItem> _items;
        // Snapshot taken at Start: the live list changes as objects are split
        // into pieces, so the run must keep its own ordering.
        private List<AutomationItem> _targets = new();

        private int _index;
        private double _phaseTime;
        private int _stuckTicks;
        private double _lastDistance = double.MaxValue;

        public CuttingController(RobotArmModel robot, IReadOnlyList<AutomationItem> items)
        {
            _robot = robot;
            _items = items;
        }

        public double SpeedMultiplier { get; set; } = 0.55;
        public double TickSeconds { get; set; } = 0.03;
        public double CutSeconds { get; set; } = 1.4;
        public double ReachTolerance { get; set; } = 0.14;

        public CuttingPhase Phase { get; private set; } = CuttingPhase.Idle;
        public string Status { get; private set; } = "Cutting: idle";
        public AutomationItem? Target { get; private set; }
        public Point3D CutPoint { get; private set; }
        public double Progress { get; private set; }
        public int ObjectsCut { get; private set; }

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
            BeginItem();
            return true;
        }

        public void Stop(string reason)
        {
            Phase = CuttingPhase.Idle;
            Target = null;
            Progress = 0;
            _stuckTicks = 0;
            SetStatus($"Cutting: stopped ({reason})");
        }

        public void ResetState()
        {
            Phase = CuttingPhase.Idle;
            Target = null;
            Progress = 0;
            ObjectsCut = 0;
            _index = 0;
            _phaseTime = 0;
            _stuckTicks = 0;
            _lastDistance = double.MaxValue;
            SetStatus("Cutting: idle");
        }

        public void Tick()
        {
            if (!IsRunning)
                return;

            _phaseTime += TickSeconds;

            switch (Phase)
            {
                case CuttingPhase.Approach:
                    if (MoveToward(HoverPoint(), $"Cutting {_index + 1}/{_targets.Count}: moving above object"))
                        Transition(CuttingPhase.Descend, $"Cutting {_index + 1}/{_targets.Count}: lowering laser");
                    break;

                case CuttingPhase.Descend:
                    if (MoveToward(CutPose(), $"Cutting {_index + 1}/{_targets.Count}: lowering laser"))
                        Transition(CuttingPhase.Cutting, $"Cutting {_index + 1}/{_targets.Count}: cutting");
                    break;

                case CuttingPhase.Cutting:
                    Progress = Math.Min(1.0, _phaseTime / CutSeconds);
                    if (Progress >= 1.0)
                    {
                        var normal = BeamDirection();
                        ObjectsCut++;
                        ItemCut?.Invoke(Target!, normal);
                        Transition(CuttingPhase.Retract, $"Cutting {_index + 1}/{_targets.Count}: object cut");
                    }
                    break;

                case CuttingPhase.Retract:
                    if (MoveToward(HoverPoint(), $"Cutting {_index + 1}/{_targets.Count}: retracting"))
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

        private void BeginItem()
        {
            Target = _targets[_index];
            CutPoint = new Point3D(Target.Position.X, Target.Position.Y + 0.16, Target.Position.Z);
            Progress = 0;
            _phaseTime = 0;
            _stuckTicks = 0;
            _lastDistance = double.MaxValue;
            Phase = CuttingPhase.Approach;
            SetStatus($"Cutting {_index + 1}/{_targets.Count}: moving above object");
        }

        private Point3D HoverPoint()
        {
            var p = Target?.Position ?? CutPoint;
            return new Point3D(p.X, p.Y + HoverHeight, p.Z);
        }

        private Point3D CutPose()
        {
            var p = Target?.Position ?? CutPoint;
            return new Point3D(p.X, p.Y + CutHeight, p.Z);
        }

        private Vector3D BeamDirection()
        {
            var delta = CutPoint - ToolTip();
            if (delta.Length < 1e-6)
                return new Vector3D(0, -1, 0);
            delta.Normalize();
            return delta;
        }

        private bool MoveToward(Point3D target, string label)
        {
            SetStatus(label);

            var ik = _robot.SolvePosition(target, 14, SpeedMultiplier);
            if (!ik.Success)
            {
                if (++_stuckTicks >= StuckTickLimit)
                    Fail("the arm cannot reach the object");
                return false;
            }

            double distance = (_robot.ForwardKinematics().EePos - target).Length;
            if (distance <= ReachTolerance)
            {
                _stuckTicks = 0;
                _lastDistance = double.MaxValue;
                return true;
            }

            if (distance > _lastDistance - ProgressEpsilon)
            {
                if (++_stuckTicks >= StuckTickLimit)
                {
                    Fail("the arm cannot reach the object");
                    return false;
                }
            }
            else
            {
                _stuckTicks = 0;
            }

            _lastDistance = distance;
            return false;
        }

        private void Transition(CuttingPhase phase, string status)
        {
            Phase = phase;
            _phaseTime = 0;
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
                fk.EePos.X + forward.X * ToolTipLength,
                fk.EePos.Y + forward.Y * ToolTipLength,
                fk.EePos.Z + forward.Z * ToolTipLength);
        }
    }
}
