using System.Diagnostics;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace RoboticArm.Tests
{
    public sealed class AutomationPlanningTests
    {
        [Fact]
        public void RoutePlanning_RunsOffTheTickThreadAndResumes()
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

            Assert.True(controller.Start());

            bool sawPlanning = false;
            double slowestPlanningTickMs = 0;
            var stopwatch = Stopwatch.StartNew();

            while (stopwatch.Elapsed < TimeSpan.FromSeconds(20))
            {
                var tick = Stopwatch.StartNew();
                controller.Tick();
                tick.Stop();

                if (controller.Phase == AutomationPhase.PlanTransport)
                {
                    sawPlanning = true;
                    slowestPlanningTickMs = Math.Max(slowestPlanningTickMs, tick.Elapsed.TotalMilliseconds);
                }

                if (controller.Phase is AutomationPhase.Lift or AutomationPhase.MoveToDrop or AutomationPhase.DropAlign)
                    break;

                if (!controller.IsRunning)
                    break;

                Thread.Sleep(2);
            }

            Assert.True(sawPlanning, "the controller never entered the planning phase");
            Assert.True(controller.IsRunning, $"automation stopped: {controller.Diagnostic}");
            Assert.NotEmpty(controller.TransportPath);
            Assert.True(slowestPlanningTickMs < 250,
                $"a tick spent {slowestPlanningTickMs:0} ms inside the planning phase");
        }
    }
}
