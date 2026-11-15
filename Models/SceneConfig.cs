namespace RoboticArm.Models
{
    public static class SceneConfig
    {
        public const double BaseSize = 1.6;
        public const double BaseHeight = 0.3;

        public const double ConveyorLength = 8.4;
        public const double ConveyorWidth = 2.4;
        public const double ConveyorThickness = 0.12;
        public const double WorkspaceHalfDepth = 2.6;

        public const double PickupZoneHalfExtent = 0.9;
        public const double PickupZoneSurfaceY = 0.30;
        public const double DropZoneSize = 1.0;
        public const double ZoneMarkerThickness = 0.025;
        public const double ZoneMarkerY = 0.13;

        public const double PickupObjectOffsetX = 0.45;
        public const double PickupObjectOffsetZ = 0.35;
        public const double PickupObjectY = 0.46;
        public const double ObjectHalfSize = 0.16;
        public const double PickupDistance = 0.55;
        public const double PickReachTolerance = 0.16;
        public const double PickApproachGap = 0.12;
        public const double PickApproachMinY = 0.60;

        public const double DropSlotOffset = 0.22;
        public const double DropSlotRadius = 0.26;
        public const double DropClearanceMargin = 0.36;
        public const double DropSettleGap = 0.10;
        public const double DropHoverGap = 0.22;
        public const double DropApproachExtra = 0.25;
        public const double DropStackClearance = 0.45;
        public const double DefaultPayloadHalf = 0.24;
        public const double EmptyCarryPayloadHalf = 0.20;

        public const double RadarMinY = 0.15;
        public const double RadarMaxY = 1.2;

        public const double PathStep = 0.14;
        public const double EeHalfExtent = 0.18;
        public const double CollisionInflate = 0.05;
        public const double FloorClearance = 0.16;
        public const double LowCarryLevel = 1.0;
        public const double ReachMargin = 0.93;
        public const double SafeHeightFloor = 0.12;
        public const double SafeHeightMargin = 0.55;
        public const double SafeHeightMin = 2.6;
        public const double TargetClearance = 0.85;
        public const double ConvergeTolerance = 0.07;
        public const double WaypointTolerance = 0.18;

        public const double DefaultSpeedMultiplier = 0.55;
        public const double AutomationBaseStepPerTick = 0.045;
        public const double TimerMilliseconds = 30.0;

        public const double CutHoverHeight = 0.75;
        public const double CutToolTipLength = 0.15;
    }
}
