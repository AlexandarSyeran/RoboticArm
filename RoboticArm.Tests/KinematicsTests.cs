using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;
using RoboticArm.Models;

namespace RoboticArm.Tests
{
    public sealed class KinematicsTests
    {
        [Fact]
        public void PositionJacobian_MatchesNumericalDerivative()
        {
            var robot = new RobotArmModel();
            var angles = (double[])robot.Angles.Clone();
            var fk = robot.ForwardKinematics(angles);
            var jacobian = InverseKinematics6D.ComputePositionJacobian(
                fk.EePos, fk.JointOrigins, fk.JointAxes);

            const double step = 0.001;
            for (int joint = 0; joint < angles.Length; joint++)
            {
                var perturbed = (double[])angles.Clone();
                perturbed[joint] += step;
                var moved = robot.ForwardKinematics(perturbed).EePos;
                var numeric = (moved - fk.EePos) / step;

                Assert.Equal(numeric.X, jacobian[0, joint], 3);
                Assert.Equal(numeric.Y, jacobian[1, joint], 3);
                Assert.Equal(numeric.Z, jacobian[2, joint], 3);
            }
        }

        [Fact]
        public void SolvePosition_ReachesNearbyTargetAndRespectsLimits()
        {
            var robot = new RobotArmModel();
            var start = robot.ForwardKinematics().EePos;
            var target = start + new Vector3D(0.6, 0.2, 0.3);

            for (int tick = 0; tick < 100; tick++)
                robot.SolvePosition(target, 14, 1.0);

            double error = (robot.ForwardKinematics().EePos - target).Length;
            Assert.True(error < 0.12, $"residual error was {error:0.000}");

            for (int i = 0; i < robot.Angles.Length; i++)
            {
                Assert.InRange(robot.Angles[i], robot.MinAngles[i], robot.MaxAngles[i]);
            }
        }

        [Fact]
        public void SolvePosition_CrossesBaseYawSeam()
        {
            var reference = new RobotArmModel();
            var referenceAngles = (double[])reference.Angles.Clone();
            referenceAngles[0] = -179.5;
            var target = reference.ForwardKinematics(referenceAngles).EePos;

            var robot = new RobotArmModel();
            robot.Angles[0] = 179.5;
            for (int tick = 0; tick < 80; tick++)
                robot.SolvePosition(target, 14, 1.0);

            double error = (robot.ForwardKinematics().EePos - target).Length;
            Assert.True(error < 0.12, $"residual error was {error:0.000}");
        }
    }
}
