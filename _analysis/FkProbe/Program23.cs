using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program23
    {
        private static string Fmt(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
        }

        public static void Run()
        {
            Console.WriteLine("=== START OPERATION DROP LAYOUT (4 BLOCKS) ===");

            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
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
                ArrivalDelay = 0.0,
                ActionDelay = 0.0,
                SpeedMultiplier = 1.0,
                TickSeconds = 0.03
            };

            if (!controller.Start())
            {
                Console.WriteLine($"start failed: '{controller.Status}'");
                return;
            }

            int tick = 0;
            while (controller.IsRunning && tick < 20000)
            {
                controller.Tick();
                tick++;
            }

            Console.WriteLine($"finished={!controller.IsRunning} status='{controller.Status}' ticks={tick}");

            double worstOverlap = 0;
            for (int i = 0; i < items.Count; i++)
            {
                var a = items[i];
                Console.WriteLine($"  item {i}: pos {Fmt(a.Position)} half {a.HalfSize.X:F2},{a.HalfSize.Y:F2},{a.HalfSize.Z:F2} completed={a.Completed}");
                for (int j = i + 1; j < items.Count; j++)
                {
                    var b = items[j];
                    double ox = Math.Min(a.Position.X + a.HalfSize.X, b.Position.X + b.HalfSize.X) -
                                Math.Max(a.Position.X - a.HalfSize.X, b.Position.X - b.HalfSize.X);
                    double oy = Math.Min(a.Position.Y + a.HalfSize.Y, b.Position.Y + b.HalfSize.Y) -
                                Math.Max(a.Position.Y - a.HalfSize.Y, b.Position.Y - b.HalfSize.Y);
                    double oz = Math.Min(a.Position.Z + a.HalfSize.Z, b.Position.Z + b.HalfSize.Z) -
                                Math.Max(a.Position.Z - a.HalfSize.Z, b.Position.Z - b.HalfSize.Z);
                    if (ox > 0 && oy > 0 && oz > 0)
                    {
                        double overlap = Math.Min(ox, Math.Min(oy, oz));
                        worstOverlap = Math.Max(worstOverlap, overlap);
                        Console.WriteLine($"  OVERLAP {i}/{j}: {overlap:F3}");
                    }
                }
            }

            Console.WriteLine($"worst overlap: {worstOverlap:F4}");
            Console.WriteLine(worstOverlap < 1e-6 ? "RESULT: start operation arranges the drop zone" : "RESULT: OVERLAP DETECTED");
        }
    }
}
