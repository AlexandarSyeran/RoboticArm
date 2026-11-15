using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program26
    {
        private static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        private static List<AutomationItem> MakeItems(double x0)
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

        private static double Penetration(Point3D a, Vector3D ha, Point3D b, Vector3D hb)
        {
            double ox = Math.Min(a.X + ha.X, b.X + hb.X) - Math.Max(a.X - ha.X, b.X - hb.X);
            double oy = Math.Min(a.Y + ha.Y, b.Y + hb.Y) - Math.Max(a.Y - ha.Y, b.Y - hb.Y);
            double oz = Math.Min(a.Z + ha.Z, b.Z + hb.Z) - Math.Max(a.Z - ha.Z, b.Z - hb.Z);
            return ox > 0 && oy > 0 && oz > 0 ? Math.Min(ox, Math.Min(oy, oz)) : 0;
        }

        private static void Scenario(string name, bool cut)
        {
            Console.WriteLine($"--- {name} ---");
            const double x0 = 4.192;
            var robot = new RobotArmModel();
            var items = MakeItems(x0);

            if (cut)
            {
                var cutting = new CuttingController(robot, items) { SpeedMultiplier = 0.55 };
                cutting.ItemCut += (item, normal) =>
                {
                    int index = items.IndexOf(item);
                    if (index < 0)
                        return;
                    var (keep, offcut) = CutPlanner.Split(item, normal);
                    items.RemoveAt(index);
                    items.Insert(index, keep);
                    items.Insert(index + 1, offcut);
                };
                cutting.Start();
                for (int t = 0; t < 8000 && cutting.IsRunning; t++)
                    cutting.Tick();
            }

            var home = new RobotArmModel().ForwardKinematics().EePos;
            var automation = new AutomationController(
                robot, items, new List<Obstacle>(),
                new Point3D(x0, 0.30, 0), new Point3D(-x0, 0.30, 0), home)
            {
                SpeedMultiplier = 1.0,
                ArrivalDelay = 0.0,
                ActionDelay = 0.05
            };

            automation.Start();

            double worstEe = 0;
            string worstAt = string.Empty;
            double belowFloor = 0;
            const double eeHalf = 0.18;

            int tick = 0;
            for (; tick < 120000 && automation.IsRunning; tick++)
            {
                automation.Tick();
                if (automation.Phase != AutomationPhase.ReturnClear &&
                    automation.Phase != AutomationPhase.ReturnHome &&
                    automation.Phase != AutomationPhase.ArrivalDelay)
                    continue;

                var fk = robot.ForwardKinematics();
                belowFloor = Math.Max(belowFloor, 0.30 - fk.EePos.Y);
                var box = new Vector3D(eeHalf, eeHalf, eeHalf);
                foreach (var item in items)
                {
                    double pen = Penetration(fk.EePos, box, item.Position, item.HalfSize);
                    if (pen > worstEe)
                    {
                        worstEe = pen;
                        worstAt = $"tick {tick} phase {automation.Phase} item {P(item.Position)} EE {P(fk.EePos)}";
                    }
                }
            }

            var diff = new double[robot.Angles.Length];
            double worstDiff = 0;
            for (int i = 0; i < diff.Length; i++)
            {
                diff[i] = robot.Angles[i] - robot.InitialFoldedAngles[i];
                if (i == 0)
                {
                    while (diff[i] > 180) diff[i] -= 360;
                    while (diff[i] < -180) diff[i] += 360;
                }
                worstDiff = Math.Max(worstDiff, Math.Abs(diff[i]));
            }

            Console.WriteLine($"  completed {items.Count(i => i.Completed)}/{items.Count}, ticks {tick}, status '{automation.Status}'");
            Console.WriteLine($"  final joints [{string.Join(", ", robot.Angles.Select(a => a.ToString("F1")))}]");
            Console.WriteLine($"  worst joint diff from start pose: {worstDiff:F3} deg");
            Console.WriteLine($"  return-path worst EE/item penetration: {worstEe:F4} {worstAt}");
            Console.WriteLine($"  return-path deepest EE below table top: {belowFloor:F4}");
            bool ok = items.Count(i => i.Completed) == items.Count && worstDiff < 0.35 &&
                      worstEe <= 0 && belowFloor <= 1e-6;
            Console.WriteLine(ok ? "RESULT: exact homing pose, clear path" : "RESULT: PROBLEM");
        }

        public static void Run()
        {
            Console.WriteLine("=== HOMING TO THE STANDBY POSE ===");
            Scenario("four whole blocks", cut: false);
            Scenario("eight cut pieces", cut: true);
        }
    }
}
