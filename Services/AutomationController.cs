using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    public sealed class AutomationController
    {
        // Low carry level used by the planner to slip under hanging obstacles.
        private const double LowCarryLevel = 1.0;
        // Reach is kept below this fraction of the geometric maximum so the
        // planner never asks for a fully stretched pose.
        private const double ReachMargin = 0.93;
        // A pose above the base makes yaw rotation useless for one tick, and
        // the IK can report "no progress" even though the next tick recovers.
        private const int MaxFailedIkTicks = 100;

        private readonly RobotArmModel _robot;
        private readonly IReadOnlyList<AutomationItem> _items;
        private readonly IReadOnlyList<Obstacle> _obstacles;
        private readonly RadarScanner _radar;
        private readonly List<Point3D> _transportPath = new();
        // Joint-space targets for the transport path, precomputed so the arm
        // follows one continuous branch instead of stalling in a folded pose.
        private readonly List<double[]> _posePlan = new();

        private bool _manualTransfer;
        private AutomationItem? _manualItem;
        private Point3D _manualLiftTarget;
        private string _routeMode = "straight";

        // Payload half extent used by the planner, taken from the attached
        // object's own dimensions (0.24 for a whole 0.32 block).
        private double _payloadHalf = 0.24;

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
            _radar = new RadarScanner(items, pickupZone);
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

            if (AttachedItem == null)
            {
                SetStatus("Manual place: no object held");
                SetDiagnostic("Pick an object before placing it.");
                return false;
            }

            var current = _robot.ForwardKinematics().EePos;
            var dropHover = new Point3D(DropTarget.X, DropTarget.Y + 0.45, DropTarget.Z);
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
            ResetManualState();
            Phase = AutomationPhase.Idle;
            Index = 0;
            PathIndex = 0;
            _transportPath.Clear();
            _posePlan.Clear();
            AttachedItem = null;
            ToolOffset = default;
            _payloadHalf = 0.24;
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

                    var pickIk = _robot.SolvePosition(PickTarget, 8, SpeedMultiplier);
                    if (!pickIk.Success)
                    {
                        if (++_failedIkTicks >= MaxFailedIkTicks)
                            Abort(pickIk.Error);
                        return;
                    }

                    _failedIkTicks = 0;

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
                    bool avoiding = _routeMode != "straight" && PathIndex < _transportPath.Count - 1;
                    SetStatus(avoiding
                        ? $"Automation: avoiding obstacle ({_routeMode})"
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
                        if (++_failedIkTicks >= MaxFailedIkTicks)
                            Abort(homeIk.Error);
                        return;
                    }

                    _failedIkTicks = 0;

                    SetStatus("Automation: returning for next object");
                    if ((_robot.ForwardKinematics().EePos - HomePosition).Length < 0.2)
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
                    if (SolveWaypoint(0.16))
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
                    if (SolveWaypoint(0.16))
                    {
                        _robot.GripperOpen = 1;
                        Transition(AutomationPhase.ManualPlaceRelease, "Manual place: releasing object");
                    }
                    break;

                case AutomationPhase.ManualPlaceRelease:
                    _robot.GripperOpen = 1;
                    SetStatus("Manual place: releasing object");
                    if (_phaseTime >= ActionDelay)
                    {
                        var carried = AttachedItem?.Position ?? DropTarget;
                        ReleaseAttached(new Point3D(DropTarget.X, carried.Y, DropTarget.Z));
                        if (_manualItem != null)
                            _manualItem.Completed = true;

                        // Move the arm off the placed object, otherwise the next
                        // manual pick starts from a low stretched pose right on
                        // the drop zone.
                        var lift = new Point3D(
                            DropTarget.X,
                            Math.Max(_robot.ForwardKinematics().EePos.Y + 0.55, DropTarget.Y + 0.45),
                            DropTarget.Z);
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
                   Math.Max(item.Position.Y + item.HalfSize.Y + 0.12, 0.60),
                   item.Position.Z);

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
            _payloadHalf = 0.24;
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
            _payloadHalf = 0.24;
        }

        // The payload rides rigidly on the wrist, so any wrist rotation makes
        // it swing out of the envelope the planner reserved. The route is
        // therefore planned, the actual swing along the resulting pose
        // sequence is measured, and the envelope is grown by that amount.
        private void BuildTransportPath()
        {
            if (_obstacles.Count == 0)
            {
                // Plain run: keep the original flow - lift, then drive straight
                // to the drop hover - instead of a planned corridor.
                _transportPath.Clear();
                _transportPath.Add(new Point3D(PickTarget.X, PickTarget.Y + 0.6, PickTarget.Z));
                _transportPath.Add(new Point3D(DropTarget.X, DropTarget.Y + 0.45, DropTarget.Z));
                _routeMode = "straight";
                PathIndex = 0;
                _posePlan.Clear();
                return;
            }

            var start = PickTarget;
            var end = new Point3D(DropTarget.X, DropTarget.Y + 0.45, DropTarget.Z);

            var first = PlanPath(start, end, carrying: true, 0);
            if (first.Mode == "unreachable")
            {
                Abort("no collision-free route to the drop zone");
                return;
            }

            LoadPath(first);

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
            string bestMode = first.Mode;
            double bestPenetration = double.MaxValue;

            for (int pass = 0; pass < 3 && _posePlan.Count == _transportPath.Count; pass++)
            {
                double penetration = PayloadPenetration();
                if (penetration < bestPenetration)
                {
                    bestPenetration = penetration;
                    bestPath = new List<Point3D>(_transportPath);
                    bestPoses = new List<double[]>(_posePlan);
                    bestMode = _routeMode;
                }

                if (penetration <= 0)
                    break;

                var offsets = ComputeWaypointOffsets();
                var hints = new List<KeyValuePair<Point3D, Vector3D>>(_transportPath.Count);
                for (int i = 0; i < _transportPath.Count; i++)
                    hints.Add(new KeyValuePair<Point3D, Vector3D>(_transportPath[i], offsets[i]));

                var candidate = PlanPath(start, end, carrying: true, 0, hints);
                if (candidate.Mode == "unreachable")
                    break;

                LoadPath(candidate);
            }

            if (bestPath != null)
            {
                _transportPath.Clear();
                _transportPath.AddRange(bestPath);
                _posePlan.Clear();
                _posePlan.AddRange(bestPoses!);
                _routeMode = bestMode;
            }
        }

        // The world offset of the payload at every planned waypoint, measured
        // along the pose plan so it reflects the wrist orientation the arm
        // will actually be in at that point.
        private List<Vector3D> ComputeWaypointOffsets()
        {
            var offsets = new List<Vector3D>(_transportPath.Count);
            var twin = new RobotArmModel();
            foreach (var pose in _posePlan)
            {
                Array.Copy(pose, twin.Angles, pose.Length);
                var fk = twin.ForwardKinematics();
                offsets.Add(twin.ToolOrientation(fk).Transform(ToolOffset));
            }

            return offsets;
        }

        // The deepest penetration of the payload (and gripper) box into any
        // obstacle along the whole executed motion, not just at the waypoints:
        // between two pose-plan entries the joints move simultaneously, so the
        // wrist - and with it the payload offset - is interpolated too.
        // Joint-space interpolation mirrors how StepTowardPose drives the arm.
        private double PayloadPenetration()
        {
            if (_posePlan.Count == 0)
                return 0;

            var twin = new RobotArmModel();
            const int samples = 8;
            double worst = 0;
            double[] previous = _posePlan[0];
            worst = Math.Max(worst, PosePenetration(twin, previous));

            for (int i = 1; i < _posePlan.Count; i++)
            {
                var pose = _posePlan[i];
                for (int s = 1; s <= samples; s++)
                {
                    double t = (double)s / samples;
                    for (int j = 0; j < pose.Length; j++)
                    {
                        double delta = NormalizeAngle(pose[j] - previous[j]);
                        twin.Angles[j] = previous[j] + delta * t;
                    }

                    worst = Math.Max(worst, PosePenetration(twin, twin.Angles));
                    if (worst > 0.3)
                        return worst;
                }

                previous = pose;
            }

            return worst;
        }

        private double PosePenetration(RobotArmModel twin, double[] angles)
        {
            Array.Copy(angles, twin.Angles, angles.Length);
            var fk = twin.ForwardKinematics();
            var offset = twin.ToolOrientation(fk).Transform(ToolOffset);
            double ee = BoxPenetration(fk.EePos.X, fk.EePos.Y, fk.EePos.Z, 0.18);
            double payload = BoxPenetration(fk.EePos.X + offset.X, fk.EePos.Y + offset.Y, fk.EePos.Z + offset.Z, _payloadHalf);
            return Math.Max(ee, payload);

            double BoxPenetration(double x, double y, double z, double half)
            {
                const double inflate = 0.05;
                double worst = 0;
                foreach (var obstacle in _obstacles)
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

            var poses = SolvePoseSequence((double[])_robot.Angles.Clone());
            poses ??= SolvePoseSequence((double[])_robot.InitialFoldedAngles.Clone());

            // Without a continuous joint solution the controller falls back to
            // solving each waypoint from wherever the arm currently is.
            if (poses != null)
                _posePlan.AddRange(poses);
        }

        private List<double[]>? SolvePoseSequence(double[] seed)
        {
            var twin = new RobotArmModel();
            Array.Copy(seed, twin.Angles, seed.Length);

            var poses = new List<double[]>(_transportPath.Count);
            foreach (var target in _transportPath)
            {
                if (!ConvergeTo(twin, target))
                    return null;

                poses.Add((double[])twin.Angles.Clone());
            }

            return poses;
        }

        private bool ConvergeTo(RobotArmModel twin, Point3D target)
        {
            // Near-stretched poses at the workspace edge converge slowly, so
            // the attempt budget is generous; planning happens once per leg.
            for (int attempt = 0; attempt < 200; attempt++)
            {
                twin.SolvePosition(target, 14, 1.0);
                if ((twin.ForwardKinematics().EePos - target).Length < 0.07)
                    return true;
            }

            return false;
        }

        private TransportPath PlanPath(Point3D start, Point3D end, bool carrying, double payloadBoost = 0,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints = null)
        {
            var offset = new Vector3D();
            double payloadHalf = 0.20;
            if (carrying)
            {
                var fk = _robot.ForwardKinematics();
                offset = _robot.ToolOrientation(fk).Transform(ToolOffset);
                payloadHalf = _payloadHalf + payloadBoost;
            }

            double reach = 0;
            foreach (var length in _robot.LinkLengths)
                reach += length;
            reach *= ReachMargin;

            double low = LowCarryLevel;
            double high = Math.Max(ComputeSafeHeight(start, end), low + 0.6);
            return TransportPathPlanner.Plan(start, end, _obstacles, offset, payloadHalf, low, high, reach, offsetHints);
        }

        // The corridor is crossed at a height that clears the tallest obstacle.
        // Whether that height is reachable is decided per grid cell by the
        // planner, so the route can climb only after it has retracted inward.
        private double ComputeSafeHeight(params Point3D[] targets)
        {
            double highestObstacle = 0.12;
            foreach (var obstacle in _obstacles)
                highestObstacle = Math.Max(highestObstacle, obstacle.Center.Y + obstacle.HalfSize.Y);

            // Cross the workspace above the base fold limit even when there is
            // nothing to avoid: a low straight line over the base is a pose
            // the arm cannot fold into.
            double safeY = Math.Max(highestObstacle + 0.55, 2.6);
            foreach (var target in targets)
                safeY = Math.Max(safeY, target.Y + 0.85);

            return safeY;
        }

        private bool SolveWaypoint(double tolerance = 0.18)
        {
            if (PathIndex >= _transportPath.Count)
                return true;

            if (_posePlan.Count == _transportPath.Count)
            {
                _failedIkTicks = 0;
                return StepTowardPose(_posePlan[PathIndex]);
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

            double remaining = (_robot.ForwardKinematics().EePos - target).Length;
            return remaining <= tolerance;
        }

        private bool StepTowardPose(double[] goal)
        {
            double rate = 2.2 * SpeedMultiplier / 0.55;
            bool reached = true;
            for (int joint = 0; joint < _robot.Angles.Length; joint++)
            {
                double delta = goal[joint] - _robot.Angles[joint];
                if (joint == 0)
                    delta = NormalizeAngle(delta);

                if (Math.Abs(delta) <= 0.3)
                {
                    // Close enough: snapping removes the asymptotic tail of
                    // the proportional approach below.
                    _robot.Angles[joint] = joint == 0
                        ? NormalizeAngle(_robot.Angles[joint] + delta)
                        : goal[joint];
                    continue;
                }

                reached = false;

                // Proportional within the last few degrees: full speed mid
                // segment, gentle arrival at each waypoint, so dense
                // waypoints no longer make the arm stop and restart.
                _robot.Angles[joint] += rate * Math.Clamp(delta / 5.0, -1.0, 1.0);

                // The base yaw spans a full turn: wrapping keeps a step across
                // the +/-180 seam on the short path instead of jamming at 180.
                if (_robot.MaxAngles[joint] - _robot.MinAngles[joint] >= 359.0)
                    _robot.Angles[joint] = NormalizeAngle(_robot.Angles[joint]);
            }

            _robot.Clamp();
            return reached;
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
            SetStatus(status);
        }

        private void Abort(string reason)
        {
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
