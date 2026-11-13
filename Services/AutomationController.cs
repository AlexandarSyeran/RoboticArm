using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    public sealed class AutomationController
    {
        private const double ObstacleClearance = 1.35;
        // Transport heights near the workspace edge are unreachable: keep the
        // safe height below this fraction of the arm's maximum reach.
        private const double ReachMargin = 0.95;

        private readonly RobotArmModel _robot;
        private readonly IReadOnlyList<AutomationItem> _items;
        private readonly IReadOnlyList<Obstacle> _obstacles;
        private readonly List<Point3D> _transportPath = new();

        public AutomationController(
            RobotArmModel robot,
            IReadOnlyList<AutomationItem> items,
            IReadOnlyList<Obstacle> obstacles,
            Point3D pickupZone,
            Point3D dropTarget,
            Point3D homePosition)
        {
            _robot = robot;
            _items = items;
            _obstacles = obstacles;
            PickupZone = pickupZone;
            DropTarget = dropTarget;
            HomePosition = homePosition;
        }

        public Point3D PickupZone { get; }
        public Point3D DropTarget { get; }
        public Point3D HomePosition { get; }

        public double ArrivalDelay { get; set; } = 2.0;
        public double ActionDelay { get; set; } = 0.7;
        public double SpeedMultiplier { get; set; } = 0.55;
        public double TickSeconds { get; set; } = 0.03;

        public AutomationPhase Phase { get; private set; } = AutomationPhase.Idle;
        public int Index { get; private set; }
        public int PathIndex { get; private set; }
        public Point3D PickTarget { get; private set; }
        public AutomationItem? AttachedItem { get; private set; }
        public Vector3D ToolOffset { get; private set; }
        public string Status { get; private set; } = "Automation: idle";
        public string Diagnostic { get; private set; } = string.Empty;

        public bool IsRunning => Phase != AutomationPhase.Idle;

        public event Action? StatusChanged;
        public event Action? DiagnosticChanged;
        public event Action? Finished;

        public bool Start()
        {
            SetDiagnostic(string.Empty);

            if (_items.Count == 0)
            {
                SetStatus("Automation: no objects");
                return false;
            }

            Index = 0;
            while (Index < _items.Count && _items[Index].Completed)
                Index++;

            if (Index >= _items.Count)
            {
                SetStatus("Automation: all objects completed");
                return false;
            }

            Transition(AutomationPhase.MoveToPick, "Automation: moving to first object");
            return true;
        }

        public void Stop(string reason)
        {
            _phaseTime = 0;
            Phase = AutomationPhase.Idle;
            SetStatus($"Automation: stopped ({reason})");
            SetDiagnostic(reason);
            Finished?.Invoke();
        }

        public void ResetState()
        {
            _phaseTime = 0;
            Phase = AutomationPhase.Idle;
            Index = 0;
            PathIndex = 0;
            _transportPath.Clear();
            AttachedItem = null;
            ToolOffset = default;
            SetStatus("Automation: idle");
            SetDiagnostic(string.Empty);
        }

        private double _phaseTime;

        public void Tick()
        {
            if (Phase == AutomationPhase.Idle)
                return;

            if (Index < 0 || Index >= _items.Count)
            {
                Abort("Automation object list changed.");
                return;
            }

            _phaseTime += TickSeconds;
            UpdateAttachedPose();

            var item = _items[Index];

            switch (Phase)
            {
                case AutomationPhase.MoveToPick:
                    PickTarget = new Point3D(
                        item.Position.X,
                        Math.Max(item.Position.Y + 0.28, 0.60),
                        item.Position.Z);

                    var pickIk = _robot.SolvePosition(PickTarget, 8, SpeedMultiplier);
                    if (!pickIk.Success)
                    {
                        Abort(pickIk.Error);
                        return;
                    }

                    SetStatus($"Automation: moving to object {Index + 1}/{_items.Count}");
                    if ((_robot.ForwardKinematics().EePos - PickTarget).Length < 0.16)
                        Transition(AutomationPhase.CloseGripper, "Automation: closing gripper");
                    break;

                case AutomationPhase.CloseGripper:
                    _robot.GripperOpen = 0;
                    SetStatus("Automation: closing gripper");
                    if (_phaseTime >= ActionDelay)
                        Transition(AutomationPhase.Attach, "Automation: attaching object");
                    break;

                case AutomationPhase.Attach:
                    if (!TryAttach(item))
                    {
                        Transition(AutomationPhase.MoveToPick, "Automation: retrying pickup");
                        break;
                    }

                    SetStatus("Automation: object attached");
                    BuildTransportPath();
                    Transition(AutomationPhase.Lift, "Automation: lifting");
                    break;

                case AutomationPhase.Lift:
                    SetStatus("Automation: lifting");
                    if (SolveWaypoint())
                    {
                        PathIndex++;
                        Transition(AutomationPhase.MoveToDrop, "Automation: moving to drop zone");
                    }
                    break;

                case AutomationPhase.MoveToDrop:
                    SetStatus(PathIndex == 1
                        ? "Automation: avoiding obstacle"
                        : "Automation: moving to drop zone");

                    if (SolveWaypoint())
                    {
                        PathIndex++;
                        if (PathIndex >= _transportPath.Count)
                            Transition(AutomationPhase.OpenGripper, "Automation: releasing object");
                    }
                    break;

                case AutomationPhase.OpenGripper:
                    _robot.GripperOpen = 1;
                    SetStatus("Automation: releasing object");
                    if (_phaseTime >= ActionDelay)
                    {
                        var carried = AttachedItem?.Position ?? DropTarget;
                        ReleaseAttached(new Point3D(DropTarget.X, carried.Y, DropTarget.Z));
                        item.Completed = true;
                        Transition(AutomationPhase.ReturnHome, "Automation: returning home");
                    }
                    break;

                case AutomationPhase.ReturnHome:
                    var homeIk = _robot.SolvePosition(HomePosition, 8, SpeedMultiplier);
                    if (!homeIk.Success)
                    {
                        Abort(homeIk.Error);
                        return;
                    }

                    SetStatus("Automation: returning for next object");
                    if ((_robot.ForwardKinematics().EePos - HomePosition).Length < 0.2)
                        Transition(AutomationPhase.ArrivalDelay, "Automation: waiting for next object");
                    break;

                case AutomationPhase.ArrivalDelay:
                    SetStatus($"Automation: waiting {Math.Max(0, ArrivalDelay - _phaseTime):0.0}s");
                    if (_phaseTime >= ArrivalDelay)
                    {
                        Index++;
                        if (Index >= _items.Count)
                        {
                            Phase = AutomationPhase.Idle;
                            SetStatus("Operation finished");
                            SetDiagnostic("Operation finished");
                            Finished?.Invoke();
                        }
                        else
                        {
                            Transition(AutomationPhase.MoveToPick, "Automation: moving to next object");
                        }
                    }
                    break;
            }
        }

        public bool TryAttach(AutomationItem item)
        {
            if (_robot.GripperOpen > 0.15)
                return false;

            var fk = _robot.ForwardKinematics();
            var distance = item.Position - fk.EePos;
            if (distance.Length > RobotArmModel.PickDistance)
                return false;

            var orientation = _robot.ToolOrientation(fk);
            var inverse = orientation;
            inverse.Invert();

            ToolOffset = inverse.Transform(distance);
            AttachedItem = item;
            item.Attached = true;
            return true;
        }

        public void AttachManually(AutomationItem item, Vector3D toolOffset)
        {
            AttachedItem = item;
            ToolOffset = toolOffset;
            item.Attached = true;
        }

        public void DetachAttached()
        {
            if (AttachedItem != null)
                AttachedItem.Attached = false;

            AttachedItem = null;
            ToolOffset = default;
        }

        public void UpdateAttachedPose()
        {
            if (AttachedItem == null)
                return;

            var fk = _robot.ForwardKinematics();
            var worldOffset = _robot.ToolOrientation(fk).Transform(ToolOffset);
            AttachedItem.Position = new Point3D(
                fk.EePos.X + worldOffset.X,
                fk.EePos.Y + worldOffset.Y,
                fk.EePos.Z + worldOffset.Z);
        }

        private void ReleaseAttached(Point3D position)
        {
            if (AttachedItem != null)
            {
                AttachedItem.Position = position;
                AttachedItem.Attached = false;
            }

            AttachedItem = null;
            ToolOffset = default;
        }

        private void BuildTransportPath()
        {
            _transportPath.Clear();

            double highestObstacle = 0.12;
            foreach (var obstacle in _obstacles)
                highestObstacle = Math.Max(highestObstacle, obstacle.Center.Y + obstacle.HalfSize.Y);

            double safeY = Math.Max(
                highestObstacle + ObstacleClearance,
                Math.Max(PickTarget.Y, DropTarget.Y) + 0.85);

            safeY = Math.Min(safeY, GetReachableTransportHeight());

            _transportPath.Add(new Point3D(PickTarget.X, safeY, PickTarget.Z));
            _transportPath.Add(new Point3D(DropTarget.X, safeY, DropTarget.Z));
            _transportPath.Add(new Point3D(DropTarget.X, DropTarget.Y + 0.45, DropTarget.Z));
            PathIndex = 0;
        }

        private double GetReachableTransportHeight()
        {
            double reach = 0;
            foreach (var length in _robot.LinkLengths)
                reach += length;

            reach *= ReachMargin;

            double radius = Math.Sqrt(PickTarget.X * PickTarget.X + PickTarget.Z * PickTarget.Z);
            double dropRadius = Math.Sqrt(DropTarget.X * DropTarget.X + DropTarget.Z * DropTarget.Z);
            radius = Math.Max(radius, dropRadius);

            double vertical = Math.Sqrt(Math.Max(0.0, reach * reach - radius * radius));
            return RobotArmModel.BaseHeight + vertical;
        }

        private bool SolveWaypoint(double tolerance = 0.18)
        {
            if (PathIndex >= _transportPath.Count)
                return true;

            var target = _transportPath[PathIndex];
            var ik = _robot.SolvePosition(target, 14, SpeedMultiplier);
            if (!ik.Success)
            {
                Abort(ik.Error);
                return false;
            }

            double remaining = (_robot.ForwardKinematics().EePos - target).Length;
            return remaining <= tolerance;
        }

        private void Transition(AutomationPhase phase, string status)
        {
            Phase = phase;
            _phaseTime = 0;
            SetStatus(status);
        }

        private void Abort(string reason)
        {
            Phase = AutomationPhase.Idle;
            SetStatus($"Automation: stopped ({reason})");
            SetDiagnostic(reason);
            Finished?.Invoke();
        }

        private void SetStatus(string status)
        {
            Status = status;
            StatusChanged?.Invoke();
        }

        private void SetDiagnostic(string diagnostic)
        {
            Diagnostic = diagnostic;
            DiagnosticChanged?.Invoke();
        }
    }
}
