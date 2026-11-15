namespace RoboticArm.Services
{
    public enum RobotOperation
    {
        Idle,
        Automation,
        Cutting,
        Trajectory,
        Wave,
        WristRotate
    }

    public sealed class OperationCoordinator
    {
        public RobotOperation Active { get; private set; } = RobotOperation.Idle;
        public bool IsIdle => Active == RobotOperation.Idle;

        public event Action? Changed;

        public bool TryBegin(RobotOperation operation)
        {
            if (!IsIdle)
                return false;

            Active = operation;
            Changed?.Invoke();
            return true;
        }

        public void End(RobotOperation operation)
        {
            if (Active != operation)
                return;

            Active = RobotOperation.Idle;
            Changed?.Invoke();
        }

        public void StopAll()
        {
            if (IsIdle)
                return;

            Active = RobotOperation.Idle;
            Changed?.Invoke();
        }

        public bool Is(RobotOperation operation) => Active == operation;
    }
}
