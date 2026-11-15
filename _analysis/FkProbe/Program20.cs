using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program20
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

        static (int cuts, bool finished, List<Vector3D> normals, int ticks) RunCutting(
            RobotArmModel robot, List<AutomationItem> items)
        {
            var cutting = new CuttingController(robot, items) { SpeedMultiplier = 0.55 };
            int cuts = 0;
            bool finished = false;
            var normals = new List<Vector3D>();

            cutting.ItemCut += (item, normal) =>
            {
                cuts++;
                normals.Add(normal);

                int index = items.IndexOf(item);
                if (index < 0)
                {
                    Console.WriteLine($"  [warn] cut item not in list");
                    return;
                }

                var (keep, offcut) = CutPlanner.Split(item, normal);
                items.RemoveAt(index);
                items.Insert(index, keep);
                items.Insert(index + 1, offcut);
            };
            cutting.Finished += () => finished = true;

            cutting.Start();
            int tick = 0;
            for (; tick < 8000 && cutting.IsRunning; tick++)
                cutting.Tick();

            Console.WriteLine($"  cutting: {cuts} cuts, finished={finished}, objectsCut={cutting.ObjectsCut}, " +
                              $"ticks {tick}, items now {items.Count}, status '{cutting.Status}'");
            return (cuts, finished, normals, tick);
        }

        public static void Run()
        {
            Console.WriteLine("=== Cutting then transporting every object ===");

            var robot = new RobotArmModel();
            double x0 = 4.192;
            var items = MakeItems(x0);

            var (cuts, finished, normals, _) = RunCutting(robot, items);
            for (int i = 0; i < normals.Count; i++)
            {
                var n = normals[i];
                double tilt = Math.Acos(Math.Abs(n.Y)) * 180.0 / Math.PI;
                Console.WriteLine($"  plane {i + 1}: {V(n)} tilt {tilt:F1} deg");
            }

            Console.WriteLine("  pieces:");
            foreach (var item in items)
            {
                Console.WriteLine($"    palette {item.PaletteIndex} side {item.CutSide} pos {P(item.Position)} " +
                                  $"half {V(item.HalfSize)}");
            }

            // Now the automatic run must transport every object, cut or not.
            var automation = new AutomationController(
                robot,
                items,
                new List<Obstacle>(),
                new Point3D(x0, 0.30, 0),
                new Point3D(-x0, 0.30, 0),
                new RobotArmModel().ForwardKinematics().EePos)
            {
                SpeedMultiplier = 1.0,
                ArrivalDelay = 0.0,
                ActionDelay = 0.05
            };

            bool autoFinished = false;
            automation.Finished += () => autoFinished = true;

            if (!automation.Start())
            {
                Console.WriteLine($"  automation start refused: '{automation.Status}' '{automation.Diagnostic}'");
                return;
            }

            int tick = 0;
            for (; tick < 120000 && automation.IsRunning; tick++)
                automation.Tick();

            int completed = items.Count(i => i.Completed);
            Console.WriteLine($"  automation: completed {completed}/{items.Count}, ticks {tick} (~{tick * 0.03:F0}s), " +
                              $"finished={autoFinished}, status '{automation.Status}', diag '{automation.Diagnostic}'");

            foreach (var item in items)
                Console.WriteLine($"    completed={item.Completed} pos {P(item.Position)}");

            // Second cutting cycle from a clean reset, as Reset Auto Objects does.
            items.Clear();
            items.AddRange(MakeItems(x0));
            robot.ResetPose();
            var cycle2 = RunCutting(robot, items);

            bool ok = cuts == 4 && finished && completed == 8 && autoFinished &&
                      cycle2.cuts == 4 && cycle2.finished && items.Count == 8;
            Console.WriteLine(ok ? "RESULT: cut -> transport all OK" : "RESULT: PROBLEM");
        }

        static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";
        static string V(Vector3D v) => $"({v.X,6:F3},{v.Y,6:F3},{v.Z,6:F3})";
    }
}
