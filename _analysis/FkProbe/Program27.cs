using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program27
    {
        static List<AutomationItem> MakeItems(double x0)
        {
            var positions = new[]
            {
                new Point3D(x0 - 0.45, 0.46, -0.35),
                new Point3D(x0 + 0.45, 0.46, -0.35),
                new Point3D(x0 - 0.45, 0.46, 0.35),
                new Point3D(x0 + 0.45, 0.46, 0.35)
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

        // Distance between two segments, used to spot links folding into each
        // other (a self-intersection the position solver never checks).
        static double SegmentDistance(Point3D a0, Point3D a1, Point3D b0, Point3D b1)
        {
            double best = double.MaxValue;
            const int samples = 12;
            for (int i = 0; i <= samples; i++)
            {
                var p = a0 + (a1 - a0) * ((double)i / samples);
                for (int j = 0; j <= samples; j++)
                {
                    var q = b0 + (b1 - b0) * ((double)j / samples);
                    best = Math.Min(best, (p - q).Length);
                }
            }

            return best;
        }

        public static void Run()
        {
            Console.WriteLine("=== LINK FOLDING WATCH (4 blocks, speed 1.0) ===");
            const double x0 = 4.192;
            var robot = new RobotArmModel();
            var items = MakeItems(x0);
            var automation = new AutomationController(
                robot, items, new List<Obstacle>(),
                new Point3D(x0, 0.30, 0), new Point3D(-x0, 0.30, 0),
                robot.ForwardKinematics().EePos)
            {
                SpeedMultiplier = 1.0,
                ArrivalDelay = 0.0,
                ActionDelay = 0.05
            };

            automation.Start();
            double worst = double.MaxValue;
            string worstAt = string.Empty;
            string lastPhase = string.Empty;
            int tick = 0;
            for (; tick < 60000 && automation.IsRunning; tick++)
            {
                automation.Tick();
                var fk = robot.ForwardKinematics();
                var o = fk.JointOrigins;
                var ee = fk.EePos;

                double d = double.MaxValue;
                d = Math.Min(d, SegmentDistance(o[1], o[2], o[3], o[4]));
                d = Math.Min(d, SegmentDistance(o[2], o[3], o[4], ee));
                d = Math.Min(d, SegmentDistance(o[1], o[2], o[4], ee));

                if (d < worst)
                {
                    worst = d;
                    worstAt = $"tick {tick} phase {automation.Phase} angles [{string.Join(", ", robot.Angles.Select(a => a.ToString("F1")))}] " +
                              $"EE ({ee.X:F2},{ee.Y:F2},{ee.Z:F2}) minLinkGap {d:F3}";
                }

                if (automation.Phase != AutomationPhase.Idle)
                    lastPhase = automation.Phase.ToString();
            }

            Console.WriteLine($"closest non-adjacent links: {worst:F3}");
            Console.WriteLine($"  {worstAt}");
            Console.WriteLine($"completed {items.Count(i => i.Completed)}/4, ticks {tick}, status '{automation.Status}'");

            // Standby pose for reference.
            robot.ResetPose();
            var standby = robot.ForwardKinematics();
            var so = standby.JointOrigins;
            double sd = double.MaxValue;
            sd = Math.Min(sd, SegmentDistance(so[1], so[2], so[3], so[4]));
            sd = Math.Min(sd, SegmentDistance(so[2], so[3], so[4], standby.EePos));
            sd = Math.Min(sd, SegmentDistance(so[1], so[2], so[4], standby.EePos));
            Console.WriteLine($"standby closest non-adjacent links: {sd:F3}");
        }
    }
}
