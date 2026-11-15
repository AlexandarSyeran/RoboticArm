using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace RoboticArm.Tests
{
    public sealed class CuttingFlowTests
    {
        [Fact]
        public void Cutting_SweepsTheBeamAcrossEveryObjectAndSplitsIt()
        {
            var robot = new RobotArmModel();
            var items = CreatePickupItems(robot);
            var controller = new CuttingController(robot, items)
            {
                SpeedMultiplier = 1.0,
                TickSeconds = 0.03
            };

            var cutNormals = new List<Vector3D>();
            var beamMinX = double.MaxValue;
            var beamMaxX = double.MinValue;
            bool sawCutting = false;

            controller.ItemCut += (_, normal) =>
            {
                cutNormals.Add(normal);
            };

            Assert.True(controller.Start(), controller.Status);

            for (int tick = 0; tick < 4000 && controller.IsRunning; tick++)
            {
                controller.Tick();

                if (controller.Phase == CuttingPhase.Cutting)
                {
                    sawCutting = true;
                    beamMinX = Math.Min(beamMinX, controller.CutPoint.X);
                    beamMaxX = Math.Max(beamMaxX, controller.CutPoint.X);
                }
            }

            Assert.True(sawCutting, "the controller never entered the cutting stroke");
            Assert.Equal(items.Count, controller.ObjectsCut);
            Assert.Equal(items.Count, cutNormals.Count);
            Assert.True(beamMaxX - beamMinX > 0.5,
                $"the beam covered only {beamMaxX - beamMinX:0.00} m along the cut line");

            foreach (var normal in cutNormals)
            {
                Assert.True(normal.Y < -0.9,
                    $"the cut plane was not horizontal: normal {normal}");
            }
        }

        [Fact]
        public void Cutting_ReachesTheCutLineFromTheFiveJointArmLimits()
        {
            var robot = new RobotArmModel();
            var items = CreatePickupItems(robot);
            var controller = new CuttingController(robot, items)
            {
                SpeedMultiplier = 1.0,
                TickSeconds = 0.03
            };

            Assert.True(controller.Start(), controller.Status);

            for (int tick = 0; tick < 4000 && controller.IsRunning; tick++)
                controller.Tick();

            for (int joint = 0; joint < robot.Angles.Length; joint++)
            {
                Assert.InRange(robot.Angles[joint], robot.MinAngles[joint], robot.MaxAngles[joint]);
            }
        }

        [Fact]
        public void Cutting_StartsFromAnArbitraryPoseAndMovesSmoothly()
        {
            var reference = new RobotArmModel();
            var robot = new RobotArmModel();
            double[] startAngles = { -70, -12, -55, 100, 25 };
            Array.Copy(startAngles, robot.Angles, startAngles.Length);
            var items = CreatePickupItems(reference);

            var controller = new CuttingController(robot, items)
            {
                SpeedMultiplier = 1.0,
                TickSeconds = 0.03
            };

            Assert.True(controller.Start(), controller.Status);

            var previousAngles = (double[])robot.Angles.Clone();
            var previousStep = new double[robot.Angles.Length];
            double largestStepChange = 0;

            for (int tick = 0; tick < 6000 && controller.IsRunning; tick++)
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
            }

            Assert.Equal(items.Count, controller.ObjectsCut);
            Assert.False(controller.IsRunning, $"the cutting run did not finish: {controller.Status}");
            Assert.True(largestStepChange < 0.75,
                $"a joint changed its per-tick step by {largestStepChange:0.000} deg");
        }

        private static List<AutomationItem> CreatePickupItems(RobotArmModel robot)
        {
            double pickupX = robot.ForwardKinematics().EePos.X;
            var positions = new[]
            {
                new Point3D(pickupX - SceneConfig.PickupObjectOffsetX, SceneConfig.PickupObjectY, -SceneConfig.PickupObjectOffsetZ),
                new Point3D(pickupX + SceneConfig.PickupObjectOffsetX, SceneConfig.PickupObjectY, -SceneConfig.PickupObjectOffsetZ),
                new Point3D(pickupX - SceneConfig.PickupObjectOffsetX, SceneConfig.PickupObjectY, SceneConfig.PickupObjectOffsetZ),
                new Point3D(pickupX + SceneConfig.PickupObjectOffsetX, SceneConfig.PickupObjectY, SceneConfig.PickupObjectOffsetZ)
            };

            var items = new List<AutomationItem>();
            for (int i = 0; i < positions.Length; i++)
            {
                items.Add(new AutomationItem
                {
                    StartPosition = positions[i],
                    Position = positions[i],
                    PaletteIndex = i
                });
            }

            return items;
        }
    }
}
