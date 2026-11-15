using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace RoboticArm.Tests
{
    public sealed class AutomationDropTests
    {
        [Fact]
        public void AutomaticRun_LowersAndReleasesWithoutJerkingThePayload()
        {
            var robot = new RobotArmModel();
            var home = robot.ForwardKinematics().EePos;
            var pickup = new Point3D(home.X, SceneConfig.PickupZoneSurfaceY, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);
            var start = new Point3D(
                pickup.X + SceneConfig.PickupObjectOffsetX,
                SceneConfig.PickupObjectY,
                pickup.Z + SceneConfig.PickupObjectOffsetZ);

            var item = new AutomationItem
            {
                StartPosition = start,
                Position = start,
                PaletteIndex = 0
            };

            var controller = new AutomationController(
                robot, new[] { item }, Array.Empty<Obstacle>(), pickup, drop, home)
            {
                ArrivalDelay = 0.1,
                ActionDelay = 0.1,
                TickSeconds = 0.03,
                SpeedMultiplier = 1.0
            };

            Assert.True(controller.Start(), controller.Diagnostic);

            bool sawDrop = false;
            bool sawRelease = false;
            double lastDropY = 0;
            double maxRise = 0;
            double maxDropStep = 0;
            double maxSettleStep = 0;
            Vector3D dropDirection = default;
            Vector3D dropEndDirection = default;
            bool capturedEndDirection = false;
            Point3D lastItemPosition = item.Position;

            for (int tick = 0; tick < 8000 && controller.IsRunning; tick++)
            {
                controller.Tick();

                if (controller.Phase == AutomationPhase.OpenGripper && !capturedEndDirection)
                {
                    dropEndDirection = robot.ToolDirection();
                    capturedEndDirection = true;
                }

                if (controller.Phase == AutomationPhase.DropAlign)
                {
                    double y = robot.ForwardKinematics().EePos.Y;
                    if (!sawDrop)
                    {
                        sawDrop = true;
                        dropDirection = robot.ToolDirection();
                    }
                    else
                    {
                        maxRise = Math.Max(maxRise, y - lastDropY);
                    }

                    if (lastDropY != 0)
                        maxDropStep = Math.Max(maxDropStep, lastDropY - y);
                    lastDropY = y;
                }

                if (controller.Phase == AutomationPhase.OpenGripper)
                {
                    sawRelease = true;
                    maxSettleStep = Math.Max(maxSettleStep, (item.Position - lastItemPosition).Length);
                }

                lastItemPosition = item.Position;

                if (controller.Phase == AutomationPhase.PlanTransport)
                    Thread.Sleep(1);
            }

            Assert.True(sawDrop, $"the run stopped before the drop phase: {controller.Diagnostic}");
            Assert.True(sawRelease, $"the run stopped before releasing the object: {controller.Diagnostic}");
            Assert.True(item.Completed, "the object was never placed");
            Assert.True(!controller.IsRunning, $"the run did not finish: {controller.Status}");

            Assert.True(maxRise < 0.02, $"the tool end effector rose {maxRise:0.000} m during the descent");
            Assert.True(maxDropStep < 0.12, $"the tool dropped {maxDropStep:0.000} m in one tick");
            Assert.True(maxSettleStep < 0.05, $"the released object jumped {maxSettleStep:0.000} m in one tick");

            var finalDirection = dropEndDirection;
            double directionDrift = Vector3D.AngleBetween(dropDirection, finalDirection);
            Assert.True(directionDrift < 8.0,
                $"the wrist did not hold the payload orientation: {directionDrift:0.0} deg");
        }

        [Fact]
        public void AutomaticRun_StartsFromAnArbitraryPoseAndMovesSmoothly()
        {
            var reference = new RobotArmModel();
            var home = reference.ForwardKinematics().EePos;
            var pickup = new Point3D(home.X, SceneConfig.PickupZoneSurfaceY, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);
            var start = new Point3D(
                pickup.X + SceneConfig.PickupObjectOffsetX,
                SceneConfig.PickupObjectY,
                pickup.Z + SceneConfig.PickupObjectOffsetZ);

            var robot = new RobotArmModel();
            double[] startAngles = { -70, -12, -55, 100, 25 };
            Array.Copy(startAngles, robot.Angles, startAngles.Length);

            var item = new AutomationItem
            {
                StartPosition = start,
                Position = start,
                PaletteIndex = 0
            };

            var controller = new AutomationController(
                robot, new[] { item }, Array.Empty<Obstacle>(), pickup, drop, home)
            {
                ArrivalDelay = 0.1,
                ActionDelay = 0.1,
                TickSeconds = 0.03,
                SpeedMultiplier = 1.0
            };

            Assert.True(controller.Start(), controller.Diagnostic);

            var previousAngles = (double[])robot.Angles.Clone();
            var previousStep = new double[robot.Angles.Length];
            double largestStepChange = 0;

            for (int tick = 0; tick < 8000 && controller.IsRunning; tick++)
            {
                controller.Tick();

                for (int joint = 0; joint < robot.Angles.Length; joint++)
                {
                    double step = robot.Angles[joint] - previousAngles[joint];
                    if (joint == 0)
                    {
                        while (step > 180) step -= 360;
                        while (step < -180) step += 360;
                    }

                    if (tick > 0)
                        largestStepChange = Math.Max(largestStepChange, Math.Abs(step - previousStep[joint]));

                    previousStep[joint] = step;
                    previousAngles[joint] = robot.Angles[joint];
                }

                if (controller.Phase == AutomationPhase.PlanTransport)
                    Thread.Sleep(1);
            }

            Assert.True(item.Completed, $"the run did not place the object: {controller.Status} / {controller.Diagnostic}");
            Assert.False(controller.IsRunning, $"the run did not finish: {controller.Status}");
            Assert.True(largestStepChange < 0.75,
                $"a joint changed its per-tick step by {largestStepChange:0.000} deg");
        }
    }
}
