using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;
using FkMath = RoboticArm.Kinematics.ForwardKinematics;

namespace RoboticArm.Models
{
    public readonly record struct FkResult(
        Point3D EePos,
        Matrix3D EeRot,
        Point3D[] JointOrigins,
        Vector3D[] JointAxes);

    public readonly record struct IkResult(bool Success, string Error, double RemainingError);

    public sealed class RobotArmModel
    {
        public const double BaseHeight = 0.3;
        public const double WristLength = 0.6;
        public const double AutomationBaseStepPerTick = 0.045;
        public const double PickDistance = 0.55;

        public double L1 { get; init; } = 2.0;
        public double L2 { get; init; } = 1.5;
        public double L3 { get; init; } = 1.0;

        public double[] Angles { get; } = { 0, 0, -45, 90, -90 };
        public double[] MinAngles { get; } = { -180, -45, -120, -150, -120 };
        public double[] MaxAngles { get; } = { 180, 45, 120, 150, 120 };
        public double[] InitialFoldedAngles { get; } = { 0, 0, -45, 90, -90 };

        public double WristRoll { get; set; }
        public double GripperOpen { get; set; } = 1.0;

        public double[] LinkLengths { get; }

        private static readonly Vector3D[] LocalAxes =
        {
            new(0, 1, 0),  // base yaw
            new(0, 0, -1), // base elevation (positive lifts the arm)
            new(0, 0, 1),  // shoulder
            new(0, 0, 1),  // elbow
            new(0, 0, 1)   // wrist
        };

        // Local (parent-frame) rotation axis of each joint, in joint order:
        // base yaw, base elevation, shoulder, elbow, wrist.
        public static IReadOnlyList<Vector3D> JointAxesLocal => LocalAxes;

        public RobotArmModel()
        {
            // The base yaw rotates in place, so it has no link of its own:
            // the first segment belongs to the base-elevation joint.
            LinkLengths = new[] { 0.0, L1, L2, L3, WristLength };
        }

        public FkResult ForwardKinematics() => ForwardKinematics(Angles);

        public FkResult ForwardKinematics(double[] angles)
        {
            int n = angles.Length;
            var jointOrigins = new Point3D[n];
            var jointAxes = new Vector3D[n];

            Matrix3D m = FkMath.Translate(0, BaseHeight / 2.0, 0);

            for (int i = 0; i < n; i++)
            {
                jointOrigins[i] = m.Transform(new Point3D(0, 0, 0));

                var axisWorld = m.Transform(LocalAxes[i]);
                axisWorld.Normalize();
                jointAxes[i] = axisWorld;

                // Rotate about the joint's own local axis. Using the world axis
                // would apply the parent rotations twice and tilt the arm out
                // of its plane as soon as the base yaw moved.
                var rotation = FkMath.RotateAxisAngle(LocalAxes[i], angles[i]);
                m = rotation * m;

                var translation = FkMath.Translate(LinkLengths[i], 0, 0);
                m = translation * m;
            }

            var eePos = m.Transform(new Point3D(0, 0, 0));
            var eeRot = m;
            eeRot.OffsetX = eeRot.OffsetY = eeRot.OffsetZ = 0;
            return new FkResult(eePos, eeRot, jointOrigins, jointAxes);
        }

        public Matrix3D ToolOrientation(FkResult fk)
            => fk.EeRot * FkMath.RotateAxisAngle(new Vector3D(1, 0, 0), WristRoll);

        public void Clamp()
        {
            for (int i = 0; i < Angles.Length; i++)
            {
                Angles[i] = Math.Max(MinAngles[i], Math.Min(MaxAngles[i], Angles[i]));
            }
        }

        public void ResetPose()
        {
            Array.Copy(InitialFoldedAngles, Angles, InitialFoldedAngles.Length);
            Clamp();
            WristRoll = 0;
            GripperOpen = 1;
        }

        public IkResult SolvePosition(
            Point3D target,
            int iterations,
            double speedMultiplier,
            Func<double[], bool>? isCandidateSafe = null)
        {
            var requestedTarget = target;
            var current = ForwardKinematics();
            double initialError = (current.EePos - requestedTarget).Length;

            var delta = target - current.EePos;
            double maxStep = AutomationBaseStepPerTick * speedMultiplier / 0.55;
            if (delta.Length > maxStep)
            {
                delta.Normalize();
                target = new Point3D(
                    current.EePos.X + delta.X * maxStep,
                    current.EePos.Y + delta.Y * maxStep,
                    current.EePos.Z + delta.Z * maxStep);
            }

            bool solved = InverseKinematics6D.SolvePosition(
                target,
                Angles,
                angles =>
                {
                    var fk = ForwardKinematics(angles);
                    return (fk.EePos, fk.JointOrigins, fk.JointAxes);
                },
                MinAngles,
                MaxAngles,
                0.001,
                iterations,
                0.12,
                isCandidateSafe);

            Clamp();

            var actualPosition = ForwardKinematics().EePos;
            double remainingError = (actualPosition - requestedTarget).Length;
            double appliedError = (actualPosition - target).Length;
            bool madeProgress = remainingError < initialError - 1e-5;
            bool success = solved || remainingError < 0.16 || madeProgress;

            var error = success
                ? string.Empty
                : $"IK made no progress; residual error {remainingError:0.000} (step error {appliedError:0.000}).";

            return new IkResult(success, error, remainingError);
        }

        public bool Solve6D(Point3D targetPos, Matrix3D targetRot, bool useOrientation, int maxIterations = 120)
        {
            bool solved = InverseKinematics6D.Solve6D(
                targetPos,
                targetRot,
                Angles,
                angles =>
                {
                    var fk = ForwardKinematics(angles);
                    return (fk.EePos, fk.EeRot, fk.JointOrigins, fk.JointAxes);
                },
                MinAngles,
                MaxAngles,
                useOrientation,
                1e-3,
                1e-2,
                maxIterations,
                0.08);

            Clamp();
            return solved;
        }

        public static Matrix3D EulerToRotation(double rxDegrees, double ryDegrees, double rzDegrees)
        {
            var rz = FkMath.RotateAxisAngle(new Vector3D(0, 0, 1), rzDegrees);
            var ry = FkMath.RotateAxisAngle(new Vector3D(0, 1, 0), ryDegrees);
            var rx = FkMath.RotateAxisAngle(new Vector3D(1, 0, 0), rxDegrees);
            return rx * ry * rz;
        }
    }
}
