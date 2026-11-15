using RoboticArm.Models;

namespace RoboticArm.Services
{
    // Acceleration-limited joint-space motion. Every workflow phase drives the
    // arm through this controller, so the velocity is continuous across phase
    // boundaries and a re-planned target can be followed without a jerk.
    public sealed class JointMotion
    {
        private const double MaxSpeedDegPerSecond = 80.0;
        private const double MaxAccelerationDegPerSecondSquared = 320.0;
        private const double ArrivalSpeed = 2.0;
        private const double SnapAngle = 0.2;
        private const double ProportionalGain = 6.0;

        private readonly double[] _velocity;

        public JointMotion(int jointCount)
        {
            _velocity = new double[jointCount];
        }

        public void Reset()
        {
            Array.Clear(_velocity, 0, _velocity.Length);
        }

        // Moves the joints toward the goal and returns the largest remaining
        // absolute joint error in degrees.
        public double Step(
            RobotArmModel robot,
            IReadOnlyList<double> goal,
            double speedMultiplier,
            double deltaSeconds)
        {
            if (deltaSeconds <= 0)
                return double.MaxValue;

            double scale = Math.Max(0.02, speedMultiplier / SceneConfig.DefaultSpeedMultiplier);
            double maxSpeed = MaxSpeedDegPerSecond * scale;
            double maxAcceleration = MaxAccelerationDegPerSecondSquared * scale;
            double maxVelocityChange = maxAcceleration * deltaSeconds;

            double remaining = 0;
            for (int i = 0; i < _velocity.Length; i++)
            {
                bool fullTurn = robot.MaxAngles[i] - robot.MinAngles[i] >= 359.0;

                double delta = goal[i] - robot.Angles[i];
                if (fullTurn)
                    delta = NormalizeAngle(delta);

                if (double.IsNaN(delta) || double.IsInfinity(delta))
                {
                    _velocity[i] = 0;
                    continue;
                }

                if (Math.Abs(delta) <= SnapAngle && Math.Abs(_velocity[i]) <= ArrivalSpeed)
                {
                    robot.Angles[i] = fullTurn
                        ? NormalizeAngle(robot.Angles[i] + delta)
                        : goal[i];
                    _velocity[i] = 0;
                    continue;
                }

                // Never command a speed that cannot be brought back to zero
                // before the goal.
                double stopSpeed = Math.Sqrt(2.0 * maxAcceleration * Math.Abs(delta));
                double desired = Math.Clamp(delta * ProportionalGain, -maxSpeed, maxSpeed);
                desired = Math.Clamp(desired, -stopSpeed, stopSpeed);

                _velocity[i] += Math.Clamp(desired - _velocity[i], -maxVelocityChange, maxVelocityChange);
                double step = _velocity[i] * deltaSeconds;
                if (Math.Abs(step) > Math.Abs(delta))
                {
                    step = delta;
                    _velocity[i] = 0;
                }

                double next = robot.Angles[i] + step;
                if (fullTurn)
                {
                    next = NormalizeAngle(next);
                }
                else
                {
                    double clamped = Math.Max(robot.MinAngles[i], Math.Min(robot.MaxAngles[i], next));
                    if (!clamped.Equals(next))
                        _velocity[i] = 0;
                    next = clamped;
                }

                robot.Angles[i] = next;

                double left = goal[i] - next;
                if (fullTurn)
                    left = NormalizeAngle(left);
                remaining = Math.Max(remaining, Math.Abs(left));
            }

            return remaining;
        }

        private static double NormalizeAngle(double degrees)
        {
            degrees %= 360.0;
            if (degrees > 180.0)
                degrees -= 360.0;
            if (degrees <= -180.0)
                degrees += 360.0;
            return degrees;
        }
    }
}
