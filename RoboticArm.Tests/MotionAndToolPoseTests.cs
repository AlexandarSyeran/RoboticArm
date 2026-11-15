using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace RoboticArm.Tests
{
    public sealed class MotionAndToolPoseTests
    {
        [Fact]
        public void JointMotion_ChangesVelocityGraduallyAndArrives()
        {
            var robot = new RobotArmModel();
            var motion = new JointMotion(robot.Angles.Length);
            var goal = (double[])robot.Angles.Clone();
            goal[2] += 40.0;

            double previousStep = 0;
            double largestStepChange = 0;
            double largestStep = 0;

            for (int tick = 0; tick < 200; tick++)
            {
                double before = robot.Angles[2];
                motion.Step(robot, goal, SceneConfig.DefaultSpeedMultiplier, SceneConfig.TimerMilliseconds / 1000.0);
                double step = robot.Angles[2] - before;
                if (tick > 0)
                    largestStepChange = Math.Max(largestStepChange, Math.Abs(step - previousStep));
                largestStep = Math.Max(largestStep, Math.Abs(step));
                previousStep = step;
            }

            Assert.True(Math.Abs(robot.Angles[2] - goal[2]) < 0.3,
                $"joint did not arrive, error {Math.Abs(robot.Angles[2] - goal[2]):0.00} deg");
            Assert.True(largestStep <= 80.0 / 0.55 * 0.03 + 0.2,
                $"per-tick step {largestStep:0.00} deg exceeded the speed limit");
            Assert.True(largestStepChange <= 320.0 / 0.55 * 0.03 * 0.03 + 0.02,
                $"per-tick step change {largestStepChange:0.000} deg exceeded the acceleration limit");
        }

        [Fact]
        public void SolveToolPose_HoldsDirectionWhileLowering()
        {
            var robot = new RobotArmModel();
            var direction = robot.ToolDirection();
            var start = robot.ForwardKinematics().EePos;
            var target = start + new Vector3D(0.18, -0.30, 0.12);

            var twin = Clone(robot);
            bool solved = twin.SolveGoal(target, direction);

            Assert.True(solved, "the direction-constrained goal was not solved");
            var result = twin.ForwardKinematics();
            double positionError = (result.EePos - target).Length;
            double directionError = Vector3D.AngleBetween(twin.ToolDirection(), direction);

            Assert.True(positionError < 0.02, $"position error {positionError:0.000}");
            Assert.True(directionError < 3.0, $"direction error {directionError:0.00} deg");
        }

        [Fact]
        public void SolveToolPose_ReachesACutPoseWithTheToolPointingDown()
        {
            var robot = new RobotArmModel();
            double pickupX = robot.ForwardKinematics().EePos.X;
            var item = new Point3D(
                pickupX - SceneConfig.PickupObjectOffsetX,
                SceneConfig.PickupObjectY,
                -SceneConfig.PickupObjectOffsetZ);
            double contactY = item.Y + SceneConfig.ObjectHalfSize + 0.02;
            var ee = new Point3D(item.X, contactY + SceneConfig.CutToolTipLength, item.Z);

            var twin = Clone(robot);
            bool solved = twin.SolveGoal(ee, new Vector3D(0, -1, 0));

            Assert.True(solved, $"the cut pose at {ee} with a vertical tool was not solved");
            double positionError = (twin.ForwardKinematics().EePos - ee).Length;
            double directionError = Vector3D.AngleBetween(twin.ToolDirection(), new Vector3D(0, -1, 0));

            Assert.True(positionError < 0.03, $"position error {positionError:0.000}");
            Assert.True(directionError < 5.0, $"direction error {directionError:0.00} deg");
        }

        private static RobotArmModel Clone(RobotArmModel source)
        {
            var twin = new RobotArmModel
            {
                L1 = source.L1,
                L2 = source.L2,
                L3 = source.L3,
                WristRoll = source.WristRoll
            };
            Array.Copy(source.Angles, twin.Angles, source.Angles.Length);
            return twin;
        }
    }
}
