using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program16
    {
        private static string Fmt(Point3D p) => $"({p.X,7:F2},{p.Y,6:F2},{p.Z,7:F2})";

        public static void Run()
        {
            Console.WriteLine("=== PLAIN RUN, NO OBSTACLES ===");
            for (int seed = 0; seed < 6; seed++)
            {
                RunSeed(seed);
            }
        }

        private static void RunSeed(int seed)
        {
            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
            // Scene zones stay anchored while the arm starts from its new pose,
            // matching MainViewModel.
            var pickup = new Point3D(4.192, 0.30, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);

            var items = new List<AutomationItem>
            {
                Item(pickup.X - 0.45, 0.46, pickup.Z - 0.35, 0),
                Item(pickup.X + 0.45, 0.46, pickup.Z - 0.35, 1),
                Item(pickup.X - 0.45, 0.46, pickup.Z + 0.35, 2),
                Item(pickup.X + 0.45, 0.46, pickup.Z + 0.35, 3)
            };

            var controller = new AutomationController(robot, items, new List<Obstacle>(), pickup, drop, folded.EePos)
            {
                ArrivalDelay = 0.3,
                ActionDelay = 0.3,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            string last = null;
            controller.Start();
            int tick = 0;
            for (; tick < 80000 && controller.IsRunning; tick++)
            {
                var before = controller.Phase;
                controller.Tick();
                if (controller.Status != last)
                {
                    last = controller.Status;
                    if (tick < 50 || controller.Status.Contains("stopped") || controller.Status.Contains("finished"))
                        Console.WriteLine($"  [seed {seed}] t={tick * 0.03:F2}s {controller.Phase} {controller.Status}");
                }

                if (!controller.IsRunning && controller.Status.Contains("stopped"))
                {
                    Console.WriteLine($"  [seed {seed}] ABORT {controller.Phase} pathIndex={controller.PathIndex}/{controller.TransportPath.Count} EE {Fmt(robot.ForwardKinematics().EePos)} angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F1")))}]");
                    return;
                }
            }

            int done = 0;
            foreach (var item in items)
                if (item.Completed) done++;
            Console.WriteLine($"  [seed {seed}] {done}/4 finished t={tick * 0.03:F1}s status='{controller.Status}'");
        }

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
        }
    }
}
