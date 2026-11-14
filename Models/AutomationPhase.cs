namespace RoboticArm.Models
{
    public enum AutomationPhase
    {
        Idle,
        MoveToPick,
        CloseGripper,
        Attach,
        Lift,
        MoveToDrop,
        OpenGripper,
        ReturnHome,
        ArrivalDelay,
        ManualPickRise,
        ManualPickApproach,
        ManualPickDescend,
        ManualPickGrip,
        ManualPickLift,
        ManualPlaceRise,
        ManualPlaceCarry,
        ManualPlaceDescend,
        ManualPlaceRelease,
        ManualPlaceRetract
    }
}
