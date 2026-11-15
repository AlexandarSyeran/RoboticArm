using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    public sealed class AutomationController
    {
        // Low carry level used by the planner to slip under hanging obstacles.
        private const double LowCarryLevel = SceneConfig.LowCarryLevel;
        // Reach is kept below this fraction of the geometric maximum so the
        // planner never asks for a fully stretched pose.
        private const double ReachMargin = SceneConfig.ReachMargin;
        // A pose above the base makes yaw rotation useless for one tick, and
        // the IK can report "no progress" even though the next tick recovers.
        private const int MaxFailedIkTicks = 100;
        // Drop-zone layout: a 2x2 grid, four objects per level; the next four
        // stack on top of them.
        private const double DropSlotOffset = SceneConfig.DropSlotOffset;
        private const double DropSlotRadius = SceneConfig.DropSlotRadius;
        // Gripper half extent plus a margin: horizontal travel near the pallet
        // happens above the stack top by at least this much.
        private const double DropClearanceMargin = SceneConfig.DropClearanceMargin;
        // The object is released this far above its slot and settles the last
        // few centimetres on release, so the open gripper stays clear of the
        // stack below instead of sinking to the object's centre.
        private const double DropSettleGap = SceneConfig.DropSettleGap;
        // Joint-space waypoints are followed early enough that the arm keeps
        // flowing through a dense pose plan instead of stopping at each one.
        private const double PathPoseToleranceDegrees = 2.5;
        private const double GoalPoseToleranceDegrees = 0.35;
        private const double GoalPositionTolerance = 0.03;
        private const double ReleaseSettleSeconds = 0.25;

        private readonly RobotArmModel _robot;
        private readonly IReadOnlyList<AutomationItem> _items;
        private readonly IReadOnlyList<Obstacle> _obstacles;
        private readonly RadarScanner _radar;
        // Internal drop-zone scan; the radar panel keeps showing the pickup zone.
        private readonly RadarScanner _dropRadar;
        private readonly List<Point3D> _transportPath = new();
        // Joint-space targets for the transport path, precomputed so the arm
        // follows one continuous branch instead of stalling in a folded pose.
        private readonly List<double[]> _posePlan = new();
        // One motion law for every phase: bounded speed and acceleration, so a
        // new goal can be picked up mid-move without the arm jerking.
        private readonly JointMotion _motion;

        private double[]? _goalPose;
        private Point3D _releaseFrom;
        private bool _releaseStarted;

        private bool _manualTransfer;
        private AutomationItem? _manualItem;
        private Point3D _manualLiftTarget;
        private Point3D _manualDropSlot;
        private Point3D _dropSlot;
        private double _returnClearY;
        private string _routeMode = "straight";

        // Payload half extent used by the planner, taken from the attached
        // object's own dimensions (0.24 for a whole 0.32 block).
        private double _payloadHalf = SceneConfig.DefaultPayloadHalf;

        // Background route search started when the object is attached.
        private Task<TransportPlanResult>? _planning;

        private sealed record TransportPlanningInput(
            Point3D Start,
            Point3D DropHover,
            Point3D Approach,
            Vector3D ToolOffset,
            double PayloadHalf,
            double[] RobotAngles,
            double L1,
            double L2,
            double L3,
            double WristRoll,
            IReadOnlyList<Obstacle> Obstacles);

        private sealed record TransportPlanResult(
            List<Point3D> Path,
            List<double[]> Poses,
            string Mode,
            string? Error);

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
            _motion = new JointMotion(robot.Angles.Length);
            PickupZone = pickupZone;
            DropTarget = dropTarget;
            HomePosition = homePosition;
            _radar = new RadarScanner(items, pickupZone);
            _dropRadar = new RadarScanner(items, dropTarget, includeCompleted: true);
        }

        public Point3D PickupZone { get; }
        public Point3D DropTarget { get; }
        public Point3D HomePosition { get; }

        public double ArrivalDelay { get; set; } = 2.0;
        public double ActionDelay { get; set; } = 0.7;
        public double SpeedMultiplier { get; set; } = SceneConfig.DefaultSpeedMultiplier;
        public double TickSeconds { get; set; } = 0.03;

        public AutomationPhase Phase { get; private set; } = AutomationPhase.Idle;
        public int Index { get; private set; }
        public int PathIndex { get; private set; }
        public IReadOnlyList<Point3D> TransportPath => _transportPath;
        public Point3D PickTarget { get; private set; }
        public AutomationItem? AttachedItem { get; private set; }
        public Vector3D ToolOffset { get; private set; }
        public string Status { get; private set; } = "Automation: idle";
        public string Diagnostic { get; private set; } = string.Empty;

        public bool IsRunning => Phase != AutomationPhase.Idle;
        public bool IsManualTransfer => Phase != AutomationPhase.Idle && _manualTransfer;

        public event Action? StatusChanged;
        public event Action? DiagnosticChanged;
        public event Action? Finished;

        public bool Start()
        {
            SetDiagnostic(string.Empty);
            _motion.Reset();
            _goalPose = null;

            if (AttachedItem != null)
            {
                SetStatus("Automation: place the held object first");
                SetDiagnostic("Place or release the held object before starting the automatic run.");
                return false;
            }

            if (_items.Count == 0)
            {
                SetStatus("Automation: no objects");
                return false;
            }

            // The arm works from the radar scan: it takes the nearest object
            // the sensor reports in the pickup zone.
            Index = FindNextItemIndex();

            if (Index < 0)
            {
                SetStatus("Automation: all objects completed");
                return false;
            }

            Transition(AutomationPhase.MoveToPick, "Automation: moving to first object");
            return true;
        }

        private int FindNextItemIndex()
        {
            var detected = _radar.Scan();
            if (detected.Count == 0)
                return -1;

            var target = detected[0].Item;
            for (int i = 0; i < _items.Count; i++)
            {
                if (ReferenceEquals(_items[i], target))
                    return i;
            }

            return -1;
        }

        public bool StartManualPick()
        {
            SetDiagnostic(string.Empty);
            _motion.Reset();
            _goalPose = null;

            if (AttachedItem != null)
            {
                SetStatus("Manual pick: gripper already holds an object");
                return false;
            }

            var item = FindAvailableItem();
            if (item == null)
            {
                SetStatus("Manual pick: no objects available");
                SetDiagnostic("No available objects in the pickup zone.");
                return false;
            }

            var current = _robot.ForwardKinematics().EePos;
            var pickTarget = PickTargetFor(item);
            var plan = PlanPath(current, pickTarget, carrying: false);
            if (plan.Mode == "unreachable")
            {
                SetStatus("Manual pick: no collision-free route");
                SetDiagnostic("No collision-free route to the pickup zone.");
                return false;
            }

            LoadPath(plan);
            _manualLiftTarget = new Point3D(pickTarget.X, plan.Points[0].Y, pickTarget.Z);

            _manualTransfer = true;
            _manualItem = item;
            _robot.GripperOpen = 1.0;
            Transition(AutomationPhase.ManualPickRise, "Manual pick: moving to pickup zone");
            return true;
        }

        public bool StartManualPlace()
        {
            SetDiagnostic(string.Empty);
            _motion.Reset();
            _goalPose = null;

            if (AttachedItem == null)
            {
                SetStatus("Manual place: no object held");
                SetDiagnostic("Pick an object before placing it.");
                return false;
            }

            var current = _robot.ForwardKinematics().EePos;
            _manualDropSlot = ChooseDropSlot(AttachedItem);
            var dropHover = DropHoverFor(_manualDropSlot, AttachedItem.HalfSize.Y);
            var plan = PlanPath(current, dropHover, carrying: true);
            if (plan.Mode == "unreachable")
            {
                SetStatus("Manual place: no collision-free route");
                SetDiagnostic("No collision-free route to the drop zone.");
                return false;
            }

            LoadPath(plan);

            _manualTransfer = true;
            _manualItem = AttachedItem;
            Transition(AutomationPhase.ManualPlaceRise, "Manual place: lifting object");
            return true;
        }

        public void Stop(string reason)
        {
            _phaseTime = 0;
            _failedIkTicks = 0;
            _planning = null;
            _motion.Reset();
            _goalPose = null;
            ResetManualState();
            Phase = AutomationPhase.Idle;
            SetStatus($"Automation: stopped ({reason})");
            SetDiagnostic(reason);
            Finished?.Invoke();
        }

        public void ResetState()
        {
            _phaseTime = 0;
            _failedIkTicks = 0;
            _planning = null;
            _motion.Reset();
            _goalPose = null;
            ResetManualState();
            Phase = AutomationPhase.Idle;
            Index = 0;
            PathIndex = 0;
            _transportPath.Clear();
            _posePlan.Clear();
            AttachedItem = null;
            ToolOffset = default;
            _payloadHalf = SceneConfig.DefaultPayloadHalf;
            SetStatus("Automation: idle");
            SetDiagnostic(string.Empty);
        }

        private void ResetManualState()
        {
            _manualTransfer = false;
            _manualItem = null;
            _manualLiftTarget = default;
        }

        private double _phaseTime;
        private int _failedIkTicks;

        public void Tick()
        {
            if (Phase == AutomationPhase.Idle)
                return;

            _phaseTime += TickSeconds;
            UpdateAttachedPose();

            if (_manualTransfer)
            {
                TickManual();
                return;
            }

            if (Index < 0 || Index >= _items.Count)
            {
                Abort("Automation object list changed.");
                return;
            }

            var item = _items[Index];

            switch (Phase)
            {
                case AutomationPhase.MoveToPick:
                    PickTarget = PickTargetFor(item);
                    SetStatus($"Automation: moving to object {Index + 1}/{_items.Count}");

                    // Plan one fixed joint goal from wherever the arm currently
                    // stands, then let the motion controller drive it there.
                    if (_goalPose == null && !PlanGoalPose(PickTarget, null, out _goalPose))
                    {
                        Abort("the arm cannot reach the object");
                        return;
                    }

                    double pickRemaining = _motion.Step(_robot, _goalPose!, SpeedMultiplier, TickSeconds);
                    if (pickRemaining <= GoalPoseToleranceDegrees ||
                        (_robot.ForwardKinematics().EePos - PickTarget).Length < SceneConfig.PickReachTolerance)
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
                    BeginTransportPlanning();
                    Transition(AutomationPhase.PlanTransport, "Automation: planning route");
                    break;

                case AutomationPhase.PlanTransport:
                    // The route search runs on a worker thread, so the tick
                    // only waits for the result instead of freezing the window.
                    if (_planning == null || !_planning.IsCompleted)
                        return;

                    var planning = _planning;
                    _planning = null;

                    TransportPlanResult plan;
                    try
                    {
                        plan = planning.GetAwaiter().GetResult();
                    }
                    catch (Exception ex)
                    {
                        Abort($"route planning failed ({ex.Message})");
                        return;
                    }

                    if (plan.Error != null)
                    {
                        Abort(plan.Error);
                        return;
                    }

                    ApplyTransportPlan(plan);
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
                    bool avoiding = _obstacles.Count > 0 && PathIndex < _transportPath.Count - 1;
                    SetStatus(avoiding
                        ? $"Automation: avoiding obstacle ({_routeMode})"
                        : "Automation: moving to drop zone");

                    if (SolveWaypoint())
                    {
                        PathIndex++;
                        if (PathIndex >= _transportPath.Count)
                            Transition(AutomationPhase.DropAlign, "Automation: lowering into the slot");
                    }
                    break;

                case AutomationPhase.DropAlign:
                    SetStatus("Automation: lowering into the slot");

                    // The goal is planned once from the hover pose and then held.
                    // Recomputing it every tick fed the wrist motion back into
                    // the target and made the payload visibly shiver on the way
                    // down.
                    if (_goalPose == null && !PlanDropPose(_dropSlot, out _goalPose))
                    {
                        Abort("the arm cannot lower the object into the slot");
                        return;
                    }

                    if (_motion.Step(_robot, _goalPose!, SpeedMultiplier, TickSeconds) > GoalPoseToleranceDegrees)
                        break;

                    Transition(AutomationPhase.OpenGripper, "Automation: releasing object");
                    break;

                case AutomationPhase.OpenGripper:
                    _robot.GripperOpen = 1;
                    SetStatus("Automation: releasing object");

                    if (!_releaseStarted)
                    {
                        _releaseStarted = true;
                        _releaseFrom = AttachedItem?.Position ?? _dropSlot;
                        ReleaseAttached(_releaseFrom);
                        _returnClearY = Math.Max(
                            DropClearanceHeight(),
                            _robot.ForwardKinematics().EePos.Y + 0.3);
                    }

                    // Let the released object settle the last few centimetres
                    // instead of teleporting it into the slot.
                    double settleT = Math.Min(1.0, _phaseTime / ReleaseSettleSeconds);
                    double settleEase = settleT * settleT * (3.0 - 2.0 * settleT);
                    item.Position = new Point3D(
                        _releaseFrom.X + (_dropSlot.X - _releaseFrom.X) * settleEase,
                        _releaseFrom.Y + (_dropSlot.Y - _releaseFrom.Y) * settleEase,
                        _releaseFrom.Z + (_dropSlot.Z - _releaseFrom.Z) * settleEase);

                    if (settleT >= 1.0)
                    {
                        item.Position = _dropSlot;
                        item.Completed = true;
                        Transition(AutomationPhase.ReturnClear, "Automation: clearing the drop zone");
                    }
                    break;

                case AutomationPhase.ReturnClear:
                    SetStatus("Automation: clearing the drop zone");

                    if (_goalPose == null)
                    {
                        var liftEe = _robot.ForwardKinematics().EePos;
                        var liftTarget = new Point3D(liftEe.X, _returnClearY, liftEe.Z);
                        if (!PlanGoalPose(liftTarget, null, out _goalPose))
                        {
                            Abort("the arm cannot clear the drop zone");
                            return;
                        }
                    }

                    if (_motion.Step(_robot, _goalPose!, SpeedMultiplier, TickSeconds) <= GoalPoseToleranceDegrees)
                        Transition(AutomationPhase.ReturnHome, "Automation: returning home");
                    break;

                case AutomationPhase.ReturnHome:
                    SetStatus("Automation: returning for next object");
                    // Home is the exact standby joint pose, not merely its
                    // end-effector position, so the arm parks in the same
                    // posture it started from.
                    if (_motion.Step(_robot, _robot.InitialFoldedAngles, SpeedMultiplier, TickSeconds) <= GoalPoseToleranceDegrees)
                        Transition(AutomationPhase.ArrivalDelay, "Automation: waiting for next object");
                    break;

                case AutomationPhase.ArrivalDelay:
                    SetStatus($"Automation: waiting {Math.Max(0, ArrivalDelay - _phaseTime):0.0}s");
                    if (_phaseTime >= ArrivalDelay)
                    {
                        Index = FindNextItemIndex();
                        if (Index < 0)
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

        private void TickManual()
        {
            switch (Phase)
            {
                case AutomationPhase.ManualPickRise:
                    SetStatus("Manual pick: moving to pickup zone");
                    if (SolveWaypoint())
                    {
                        PathIndex++;
                        Transition(AutomationPhase.ManualPickApproach, "Manual pick: moving above object");
                    }
                    break;

                case AutomationPhase.ManualPickApproach:
                    SetStatus("Manual pick: moving above object");
                    if (PathIndex >= _transportPath.Count - 1)
                    {
                        _robot.GripperOpen = 1;
                        Transition(AutomationPhase.ManualPickDescend, "Manual pick: lowering to object");
                    }
                    else if (SolveWaypoint())
                    {
                        PathIndex++;
                    }
                    break;

                case AutomationPhase.ManualPickDescend:
                    SetStatus("Manual pick: lowering to object");
                    if (SolveWaypoint(SceneConfig.PickReachTolerance))
                    {
                        _robot.GripperOpen = 0;
                        Transition(AutomationPhase.ManualPickGrip, "Manual pick: closing gripper");
                    }
                    break;

                case AutomationPhase.ManualPickGrip:
                    _robot.GripperOpen = 0;
                    SetStatus("Manual pick: closing gripper");
                    if (_phaseTime >= ActionDelay)
                    {
                        if (_manualItem == null || !TryAttach(_manualItem))
                        {
                            Abort("Manual pick: object out of reach.");
                            return;
                        }

                        _phaseTime = 0;
                        LoadPath(new TransportPath(new List<Point3D> { _manualLiftTarget }, "straight"));
                        Transition(AutomationPhase.ManualPickLift, "Manual pick: lifting object");
                    }
                    break;

                case AutomationPhase.ManualPickLift:
                    SetStatus("Manual pick: lifting object");
                    if (SolveWaypoint())
                        FinishManual("Manual pick: object held - move the arm or press Place Object");
                    break;

                case AutomationPhase.ManualPlaceRise:
                    SetStatus("Manual place: lifting object");
                    if (SolveWaypoint())
                    {
                        PathIndex++;
                        Transition(AutomationPhase.ManualPlaceCarry, "Manual place: moving to drop zone");
                    }
                    break;

                case AutomationPhase.ManualPlaceCarry:
                    SetStatus("Manual place: moving to drop zone");
                    if (PathIndex >= _transportPath.Count - 1)
                    {
                        Transition(AutomationPhase.ManualPlaceDescend, "Manual place: lowering into drop zone");
                    }
                    else if (SolveWaypoint())
                    {
                        PathIndex++;
                    }
                    break;

                case AutomationPhase.ManualPlaceDescend:
                    SetStatus("Manual place: lowering into drop zone");
                    if (SolveWaypoint(SceneConfig.PickReachTolerance))
                        Transition(AutomationPhase.ManualPlaceLower, "Manual place: lowering into the slot");
                    break;

                case AutomationPhase.ManualPlaceLower:
                    SetStatus("Manual place: lowering into the slot");
                    if (_goalPose == null && !PlanDropPose(_manualDropSlot, out _goalPose))
                    {
                        Abort("Manual place: the arm cannot lower the object into the slot.");
                        return;
                    }

                    if (_motion.Step(_robot, _goalPose!, SpeedMultiplier, TickSeconds) <= GoalPoseToleranceDegrees)
                    {
                        _robot.GripperOpen = 1;
                        Transition(AutomationPhase.ManualPlaceRelease, "Manual place: releasing object");
                    }
                    break;

                case AutomationPhase.ManualPlaceRelease:
                    _robot.GripperOpen = 1;
                    SetStatus("Manual place: releasing object");

                    if (!_releaseStarted)
                    {
                        _releaseStarted = true;
                        _releaseFrom = AttachedItem?.Position ?? _manualDropSlot;
                        ReleaseAttached(_releaseFrom);
                    }

                    double manualSettleT = Math.Min(1.0, _phaseTime / ReleaseSettleSeconds);
                    double manualEase = manualSettleT * manualSettleT * (3.0 - 2.0 * manualSettleT);
                    if (_manualItem != null)
                    {
                        _manualItem.Position = new Point3D(
                            _releaseFrom.X + (_manualDropSlot.X - _releaseFrom.X) * manualEase,
                            _releaseFrom.Y + (_manualDropSlot.Y - _releaseFrom.Y) * manualEase,
                            _releaseFrom.Z + (_manualDropSlot.Z - _releaseFrom.Z) * manualEase);
                    }

                    if (manualSettleT >= 1.0)
                    {
                        if (_manualItem != null)
                        {
                            _manualItem.Position = _manualDropSlot;
                            _manualItem.Completed = true;
                        }

                        // Move the arm off the placed object, otherwise the next
                        // manual pick starts from a low stretched pose right on
                        // the drop zone.
                        var lift = new Point3D(
                            _manualDropSlot.X,
                            Math.Max(DropClearanceHeight(), _manualDropSlot.Y + SceneConfig.DropStackClearance),
                            _manualDropSlot.Z);
                        LoadPath(new TransportPath(new List<Point3D> { lift }, "straight"));
                        Transition(AutomationPhase.ManualPlaceRetract, "Manual place: lifting away from the drop zone");
                    }
                    break;

                case AutomationPhase.ManualPlaceRetract:
                    SetStatus("Manual place: lifting away from the drop zone");
                    if (SolveWaypoint())
                        FinishManual("Manual place: object placed in drop zone");
                    break;
            }
        }

        private void FinishManual(string status)
        {
            ResetManualState();
            _motion.Reset();
            _goalPose = null;
            _transportPath.Clear();
            _posePlan.Clear();
            PathIndex = 0;
            Phase = AutomationPhase.Idle;
            SetStatus(status);
            SetDiagnostic(string.Empty);
            Finished?.Invoke();
        }

        private AutomationItem? FindAvailableItem()
        {
            AutomationItem? best = null;
            double bestDistance = double.MaxValue;
            foreach (var item in _items)
            {
                if (item.Completed || item.Attached)
                    continue;

                double distance = (item.Position - PickupZone).Length;
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = item;
                }
            }

            return best;
        }

        // Hover height above the object follows its actual half height, so
        // small cut pieces are approached lower than full blocks.
        private static Point3D PickTargetFor(AutomationItem item)
            => new(item.Position.X,
                   Math.Max(item.Position.Y + item.HalfSize.Y + SceneConfig.PickApproachGap,
                            SceneConfig.PickApproachMinY),
                   item.Position.Z);

        // The arm arranges the drop zone like a pallet: a 2x2 grid, four
        // objects per level, the next four on top. Already placed objects come
        // from the internal drop scan, so a new object lands beside or above
        // them instead of inside them.
        private Point3D ChooseDropSlot(AutomationItem? item)
        {
            double halfY = item?.HalfSize.Y ?? SceneConfig.ObjectHalfSize;
            var placed = _dropRadar.Scan();
            double[] offsets = { -DropSlotOffset, DropSlotOffset };
            var best = new Point3D(
                DropTarget.X - DropSlotOffset,
                DropTarget.Y + halfY,
                DropTarget.Z - DropSlotOffset);
            int bestCount = int.MaxValue;
            double bestTop = DropTarget.Y;

            foreach (double dx in offsets)
            {
                foreach (double dz in offsets)
                {
                    double x = DropTarget.X + dx;
                    double z = DropTarget.Z + dz;
                    int count = 0;
                    double top = DropTarget.Y;

                    foreach (var detection in placed)
                    {
                        if (Math.Abs(detection.Position.X - x) <= DropSlotRadius &&
                            Math.Abs(detection.Position.Z - z) <= DropSlotRadius)
                        {
                            count++;
                            top = Math.Max(top, detection.Position.Y + detection.HalfSize.Y);
                        }
                    }

                    if (count < bestCount)
                    {
                        bestCount = count;
                        bestTop = top;
                        best = new Point3D(x, top + halfY, z);
                    }
                }
            }

            return best;
        }

        private static Point3D DropHoverFor(Point3D slot, double halfY)
            => new(slot.X, slot.Y + halfY + SceneConfig.DropHoverGap, slot.Z);

        // Height that clears everything already resting on the pallet. The
        // final approach to a slot comes straight down from here, and the arm
        // climbs back to it after releasing, so neither the carried object nor
        // the gripper sweeps through a neighbouring piece.
        private double DropClearanceHeight()
        {
            double top = DropTarget.Y;
            foreach (var detection in _dropRadar.Scan())
                top = Math.Max(top, detection.Position.Y + detection.HalfSize.Y);
            return top + DropClearanceMargin;
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
            _payloadHalf = PayloadHalfFor(item);
            AttachedItem = item;
            item.Attached = true;
            return true;
        }

        private static double PayloadHalfFor(AutomationItem item)
            => Math.Max(item.HalfSize.X, Math.Max(item.HalfSize.Y, item.HalfSize.Z)) + 0.08;

        public void AttachManually(AutomationItem item, Vector3D toolOffset)
        {
            AttachedItem = item;
            ToolOffset = toolOffset;
            _payloadHalf = PayloadHalfFor(item);
            item.Attached = true;
        }

        public void DetachAttached()
        {
            if (AttachedItem != null)
                AttachedItem.Attached = false;

            AttachedItem = null;
            ToolOffset = default;
            _payloadHalf = SceneConfig.DefaultPayloadHalf;
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
            _payloadHalf = SceneConfig.DefaultPayloadHalf;
        }

        // The payload rides rigidly on the wrist, so any wrist rotation makes
        // it swing out of the envelope the planner reserved. The route is
        // therefore planned, the actual swing along the resulting pose
        // sequence is measured, and the envelope is grown by that amount.
        // The search is expensive (several A* passes plus a full pose
        // sequence), so it runs on a worker thread and the tick waits for the
        // finished plan instead of blocking the window.
        private void BeginTransportPlanning()
        {
            _dropSlot = ChooseDropSlot(AttachedItem);
            var dropHover = DropHoverFor(_dropSlot, AttachedItem?.HalfSize.Y ?? SceneConfig.ObjectHalfSize);
            var approach = new Point3D(
                dropHover.X,
                Math.Max(DropClearanceHeight(), dropHover.Y + SceneConfig.DropApproachExtra),
                dropHover.Z);

            var input = new TransportPlanningInput(
                PickTarget,
                dropHover,
                approach,
                ToolOffset,
                _payloadHalf,
                (double[])_robot.Angles.Clone(),
                _robot.L1,
                _robot.L2,
                _robot.L3,
                _robot.WristRoll,
                _obstacles.ToArray());

            _planning = Task.Run(() => ComputeTransportPlan(input));
        }

        private void ApplyTransportPlan(TransportPlanResult plan)
        {
            _transportPath.Clear();
            _transportPath.AddRange(plan.Path);
            _posePlan.Clear();
            _posePlan.AddRange(plan.Poses);
            _routeMode = plan.Mode;
            PathIndex = 0;
        }

        private static TransportPlanResult ComputeTransportPlan(TransportPlanningInput input)
        {
            var robot = new RobotArmModel
            {
                L1 = input.L1,
                L2 = input.L2,
                L3 = input.L3,
                WristRoll = input.WristRoll
            };
            Array.Copy(input.RobotAngles, robot.Angles, input.RobotAngles.Length);

            if (input.Obstacles.Count == 0)
            {
                // Plain run: lift, swing around the base and lower onto the
                // pallet. The swing follows a yaw arc instead of a straight
                // line between the zones: the straight line passes right over
                // the base at carrying height, where the only reachable poses
                // fold the links into each other. The arc keeps the arm
                // extended for the whole crossing.
                var lift = new Point3D(input.Start.X, input.Start.Y + 0.6, input.Start.Z);
                var arc = new List<Point3D>(Densify(ArcRoute(lift, input.Approach)));

                // The lateral travel ends at the clearance point. The last leg
                // is left to DropAlign, which steers by the measured payload
                // offset, so a tilted wrist does not drag the object down over
                // a neighbouring piece on the way in.
                return new TransportPlanResult(arc, SolvePosePlan(robot, arc) ?? new List<double[]>(), "arc", null);
            }

            var first = PlanPath(robot, input.Obstacles, input.ToolOffset, input.PayloadHalf,
                input.Start, input.Approach, carrying: true);
            if (first.Mode == "unreachable")
            {
                return new TransportPlanResult(
                    new List<Point3D>(), new List<double[]>(), first.Mode,
                    "no collision-free route to the drop zone");
            }

            var path = LoadedPath(first.Points, input.DropHover);
            var poses = SolvePosePlan(robot, path) ?? new List<double[]>();
            string mode = first.Mode;

            // The payload hangs off the tool, so its world offset swings with
            // the wrist orientation along the route - a constant planning
            // offset can clip a corner even when every planned pose is clear.
            // Each planned route is therefore scored against the offsets
            // measured along its own pose plan (joint-interpolated between
            // waypoints, mirroring how the arm will actually move) and, while
            // it clips, the next route is planned with those measured offsets
            // as per-position hints. The least-clipping route is kept.
            List<Point3D>? bestPath = null;
            List<double[]>? bestPoses = null;
            string bestMode = mode;
            double bestPenetration = double.MaxValue;

            for (int pass = 0; pass < 3 && poses.Count == path.Count; pass++)
            {
                double penetration = PayloadPenetration(
                    poses, robot, input.Obstacles, input.ToolOffset, input.PayloadHalf);
                if (penetration < bestPenetration)
                {
                    bestPenetration = penetration;
                    bestPath = new List<Point3D>(path);
                    bestPoses = new List<double[]>(poses);
                    bestMode = mode;
                }

                if (penetration <= 0)
                    break;

                var offsets = ComputeWaypointOffsets(poses, robot, input.ToolOffset);
                var hints = new List<KeyValuePair<Point3D, Vector3D>>(path.Count);
                for (int i = 0; i < path.Count; i++)
                    hints.Add(new KeyValuePair<Point3D, Vector3D>(path[i], offsets[i]));

                var candidate = PlanPath(robot, input.Obstacles, input.ToolOffset, input.PayloadHalf,
                    input.Start, input.Approach, carrying: true, 0, hints);
                if (candidate.Mode == "unreachable")
                    break;

                path = LoadedPath(candidate.Points, input.DropHover);
                poses = SolvePosePlan(robot, path) ?? new List<double[]>();
                mode = candidate.Mode;
            }

            if (bestPath != null)
            {
                path = bestPath;
                poses = bestPoses!;
                mode = bestMode;
            }

            // The planned route ends at the clearance point; the last leg drops
            // straight onto the slot. A sideways arrival would sweep the held
            // object through the neighbouring stack.
            return new TransportPlanResult(path, poses, mode, null);
        }

        private static List<Point3D> LoadedPath(IReadOnlyList<Point3D> points, Point3D dropHover)
        {
            var path = new List<Point3D>(Densify(points));
            path.Add(dropHover);
            return path;
        }

        // The world offset of the payload at every planned waypoint, measured
        // along the pose plan so it reflects the wrist orientation the arm
        // will actually be in at that point.
        private static List<Vector3D> ComputeWaypointOffsets(
            IReadOnlyList<double[]> poses, RobotArmModel robot, Vector3D toolOffset)
        {
            var offsets = new List<Vector3D>(poses.Count);
            var twin = NewTwin(robot);
            foreach (var pose in poses)
            {
                Array.Copy(pose, twin.Angles, pose.Length);
                var fk = twin.ForwardKinematics();
                offsets.Add(twin.ToolOrientation(fk).Transform(toolOffset));
            }

            return offsets;
        }

        // The deepest penetration of the payload (and gripper) box into any
        // obstacle along the whole executed motion, not just at the waypoints:
        // between two pose-plan entries the joints move simultaneously, so the
        // wrist - and with it the payload offset - is interpolated too.
        // Joint-space interpolation mirrors how StepTowardPose drives the arm.
        private static double PayloadPenetration(
            IReadOnlyList<double[]> posePlan, RobotArmModel robot,
            IReadOnlyList<Obstacle> obstacles, Vector3D toolOffset, double payloadHalf)
        {
            if (posePlan.Count == 0)
                return 0;

            var twin = NewTwin(robot);
            const int samples = 8;
            double worst = 0;
            double[] previous = posePlan[0];
            worst = Math.Max(worst, PosePenetration(twin, previous, obstacles, toolOffset, payloadHalf));

            for (int i = 1; i < posePlan.Count; i++)
            {
                var pose = posePlan[i];
                for (int s = 1; s <= samples; s++)
                {
                    double t = (double)s / samples;
                    for (int j = 0; j < pose.Length; j++)
                    {
                        double delta = NormalizeAngle(pose[j] - previous[j]);
                        twin.Angles[j] = previous[j] + delta * t;
                    }

                    worst = Math.Max(worst, PosePenetration(twin, twin.Angles, obstacles, toolOffset, payloadHalf));
                    if (worst > 0.3)
                        return worst;
                }

                previous = pose;
            }

            return worst;
        }

        private static double PosePenetration(
            RobotArmModel twin, double[] angles, IReadOnlyList<Obstacle> obstacles,
            Vector3D toolOffset, double payloadHalf)
        {
            Array.Copy(angles, twin.Angles, angles.Length);
            var fk = twin.ForwardKinematics();
            var offset = twin.ToolOrientation(fk).Transform(toolOffset);
            double ee = BoxPenetration(fk.EePos.X, fk.EePos.Y, fk.EePos.Z, SceneConfig.EeHalfExtent);
            double payload = BoxPenetration(fk.EePos.X + offset.X, fk.EePos.Y + offset.Y, fk.EePos.Z + offset.Z, payloadHalf);
            return Math.Max(ee, payload);

            double BoxPenetration(double x, double y, double z, double half)
            {
                const double inflate = 0.05;
                double worst = 0;
                foreach (var obstacle in obstacles)
                {
                    var c = obstacle.Center;
                    var s = obstacle.HalfSize;
                    double ox = Math.Min(x + half, c.X + s.X + inflate) - Math.Max(x - half, c.X - s.X - inflate);
                    double oy = Math.Min(y + half, c.Y + s.Y + inflate) - Math.Max(y - half, c.Y - s.Y - inflate);
                    double oz = Math.Min(z + half, c.Z + s.Z + inflate) - Math.Max(z - half, c.Z - s.Z - inflate);
                    if (ox > 0 && oy > 0 && oz > 0)
                        worst = Math.Max(worst, Math.Min(ox, Math.Min(oy, oz)));
                }

                return worst;
            }
        }

        private void LoadPath(TransportPath plan)
        {
            _transportPath.Clear();
            _transportPath.AddRange(Densify(plan.Points));
            _routeMode = plan.Mode;
            PathIndex = 0;

            BuildPosePlan();
        }

        // Swing between the zones on a constant-bearing circle around the
        // base, interpolating radius and height between the two ends. Crossing
        // above the base itself would force a tucked, self-overlapping pose.
        private static List<Point3D> ArcRoute(Point3D from, Point3D to)
        {
            var points = new List<Point3D> { from };
            double startYaw = Math.Atan2(from.Z, from.X);
            double endYaw = Math.Atan2(to.Z, to.X);
            double sweep = NormalizeAngle((endYaw - startYaw) * 180.0 / Math.PI) * Math.PI / 180.0;
            double startRadius = Math.Sqrt(from.X * from.X + from.Z * from.Z);
            double endRadius = Math.Sqrt(to.X * to.X + to.Z * to.Z);
            int steps = Math.Max(2, (int)Math.Ceiling(Math.Abs(sweep) / (5.0 * Math.PI / 180.0)));
            for (int i = 1; i < steps; i++)
            {
                double t = (double)i / steps;
                double yaw = startYaw + sweep * t;
                double radius = startRadius + (endRadius - startRadius) * t;
                points.Add(new Point3D(
                    radius * Math.Cos(yaw),
                    from.Y + (to.Y - from.Y) * t,
                    radius * Math.Sin(yaw)));
            }

            points.Add(to);
            return points;
        }

        // Long waypoint hops let the position solver stall in a folded pose,
        // so the route is split into short steps it can track.
        private static IEnumerable<Point3D> Densify(IReadOnlyList<Point3D> points)
        {
            const double maxGap = 0.25;
            yield return points[0];

            for (int i = 1; i < points.Count; i++)
            {
                var from = points[i - 1];
                var to = points[i];
                int steps = Math.Max(1, (int)Math.Ceiling((to - from).Length / maxGap));
                for (int step = 1; step <= steps; step++)
                    yield return from + (to - from) * ((double)step / steps);
            }
        }

        private void BuildPosePlan()
        {
            _posePlan.Clear();
            if (_transportPath.Count == 0)
                return;

            var poses = SolvePosePlan(_robot, _transportPath);

            // Without a continuous joint solution the controller falls back to
            // solving each waypoint from wherever the arm currently is.
            if (poses != null)
                _posePlan.AddRange(poses);
        }

        private static List<double[]>? SolvePosePlan(RobotArmModel robot, IReadOnlyList<Point3D> path)
        {
            if (path.Count == 0)
                return null;

            var poses = SolvePoseSequence(robot, path, robot.Angles);
            return poses ?? SolvePoseSequence(robot, path, robot.InitialFoldedAngles);
        }

        private static List<double[]>? SolvePoseSequence(
            RobotArmModel robot, IReadOnlyList<Point3D> path, double[] seed)
        {
            var twin = NewTwin(robot);
            Array.Copy(seed, twin.Angles, seed.Length);

            var poses = new List<double[]>(path.Count);
            foreach (var target in path)
            {
                if (!ConvergeTo(twin, target))
                    return null;

                poses.Add((double[])twin.Angles.Clone());
            }

            return poses;
        }

        private static RobotArmModel NewTwin(RobotArmModel robot) => new()
        {
            L1 = robot.L1,
            L2 = robot.L2,
            L3 = robot.L3,
            WristRoll = robot.WristRoll
        };

        private static bool ConvergeTo(RobotArmModel twin, Point3D target)
        {
            // Near-stretched poses at the workspace edge converge slowly, so
            // the attempt budget is generous; planning happens once per leg.
            for (int attempt = 0; attempt < 200; attempt++)
            {
                twin.SolvePosition(target, 14, 1.0);
                if ((twin.ForwardKinematics().EePos - target).Length < SceneConfig.ConvergeTolerance)
                    return true;
            }

            return false;
        }

        private TransportPath PlanPath(Point3D start, Point3D end, bool carrying, double payloadBoost = 0,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints = null)
            => PlanPath(_robot, _obstacles, ToolOffset, _payloadHalf,
                start, end, carrying, payloadBoost, offsetHints);

        private static TransportPath PlanPath(
            RobotArmModel robot, IReadOnlyList<Obstacle> obstacles, Vector3D toolOffset, double carriedPayloadHalf,
            Point3D start, Point3D end, bool carrying, double payloadBoost = 0,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints = null)
        {
            var offset = new Vector3D();
            double payloadHalf = SceneConfig.EmptyCarryPayloadHalf;
            if (carrying)
            {
                var fk = robot.ForwardKinematics();
                offset = robot.ToolOrientation(fk).Transform(toolOffset);
                payloadHalf = carriedPayloadHalf + payloadBoost;
            }

            double reach = 0;
            foreach (var length in robot.LinkLengths)
                reach += length;
            reach *= ReachMargin;

            double low = LowCarryLevel;
            double high = Math.Max(ComputeSafeHeight(obstacles, start, end), low + 0.6);
            return TransportPathPlanner.Plan(start, end, obstacles, offset, payloadHalf, low, high, reach, offsetHints);
        }

        // The corridor is crossed at a height that clears the tallest obstacle.
        // Whether that height is reachable is decided per grid cell by the
        // planner, so the route can climb only after it has retracted inward.
        private static double ComputeSafeHeight(IReadOnlyList<Obstacle> obstacles, params Point3D[] targets)
        {
            double highestObstacle = SceneConfig.SafeHeightFloor;
            foreach (var obstacle in obstacles)
                highestObstacle = Math.Max(highestObstacle, obstacle.Center.Y + obstacle.HalfSize.Y);

            // Cross the workspace above the base fold limit even when there is
            // nothing to avoid: a low straight line over the base is a pose
            // the arm cannot fold into.
            double safeY = Math.Max(highestObstacle + SceneConfig.SafeHeightMargin, SceneConfig.SafeHeightMin);
            foreach (var target in targets)
                safeY = Math.Max(safeY, target.Y + SceneConfig.TargetClearance);

            return safeY;
        }

        private bool SolveWaypoint(double tolerance = SceneConfig.WaypointTolerance)
        {
            if (PathIndex >= _transportPath.Count)
                return true;

            if (_posePlan.Count == _transportPath.Count)
            {
                _failedIkTicks = 0;
                double remaining = _motion.Step(_robot, _posePlan[PathIndex], SpeedMultiplier, TickSeconds);
                return remaining <= PathPoseToleranceDegrees;
            }

            var target = _transportPath[PathIndex];
            var ik = _robot.SolvePosition(target, 14, SpeedMultiplier);
            if (!ik.Success)
            {
                if (++_failedIkTicks >= MaxFailedIkTicks)
                    Abort(ik.Error);
                return false;
            }

            _failedIkTicks = 0;

            double distance = (_robot.ForwardKinematics().EePos - target).Length;
            return distance <= tolerance;
        }

        // Plans a whole joint goal from the current pose (and, if that pose is
        // stuck in a local minimum, from the standby pose) so a phase never
        // chases a target tick by tick.
        private bool PlanGoalPose(Point3D target, Vector3D? toolDirection, out double[] goal)
        {
            if (TryPlanGoal(_robot.Angles, target, toolDirection, out goal))
                return true;

            return TryPlanGoal(_robot.InitialFoldedAngles, target, toolDirection, out goal);
        }

        private bool TryPlanGoal(double[] seed, Point3D target, Vector3D? toolDirection, out double[] goal)
        {
            var twin = NewTwin(_robot);
            Array.Copy(seed, twin.Angles, seed.Length);

            bool solved = twin.SolveGoal(target, toolDirection);
            goal = (double[])twin.Angles.Clone();
            return solved && (twin.ForwardKinematics().EePos - target).Length <= GoalPositionTolerance;
        }

        // The drop goal is fixed before the descent starts. Holding the tool
        // direction that the payload was carried with keeps its world offset
        // constant, so the object stays under the wrist all the way down.
        private bool PlanDropPose(Point3D slot, out double[] goal)
        {
            goal = Array.Empty<double>();
            var settle = new Point3D(slot.X, slot.Y + DropSettleGap, slot.Z);
            var heldDirection = _robot.ToolDirection();
            var fk = _robot.ForwardKinematics();
            var offset = _robot.ToolOrientation(fk).Transform(ToolOffset);
            var eeTarget = settle - offset;

            if (TryPlanDrop(_robot.Angles, eeTarget, heldDirection, settle, out goal))
                return true;

            return TryPlanDrop(_robot.InitialFoldedAngles, eeTarget, heldDirection, settle, out goal);
        }

        private bool TryPlanDrop(double[] seed, Point3D eeTarget, Vector3D heldDirection, Point3D settle, out double[] goal)
        {
            var twin = NewTwin(_robot);
            Array.Copy(seed, twin.Angles, seed.Length);

            double[]? best = null;
            double bestPayloadError = double.MaxValue;
            bool solved = true;

            for (int attempt = 0; attempt < 2; attempt++)
            {
                solved = twin.SolveGoal(eeTarget, heldDirection);
                if (!solved)
                    solved = twin.SolveGoal(eeTarget, null);
                if (!solved)
                    break;

                var fk = twin.ForwardKinematics();
                var payload = fk.EePos + twin.ToolOrientation(fk).Transform(ToolOffset);
                double payloadError = (settle - payload).Length;
                if (payloadError < bestPayloadError)
                {
                    bestPayloadError = payloadError;
                    best = (double[])twin.Angles.Clone();
                }

                if (payloadError <= 0.015)
                    break;

                eeTarget += settle - payload;
            }

            if (best == null)
            {
                goal = (double[])twin.Angles.Clone();
                return false;
            }

            goal = best;
            return bestPayloadError <= 0.05;
        }

        private static double NormalizeAngle(double degrees)
        {
            while (degrees > 180.0) degrees -= 360.0;
            while (degrees < -180.0) degrees += 360.0;
            return degrees;
        }

        private void Transition(AutomationPhase phase, string status)
        {
            Phase = phase;
            _phaseTime = 0;
            _failedIkTicks = 0;
            _goalPose = null;
            _releaseStarted = false;
            SetStatus(status);
        }

        private void Abort(string reason)
        {
            _planning = null;
            _motion.Reset();
            _goalPose = null;
            ResetManualState();
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
