using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    // Investigates why the transport pose plan folds the arm into itself.
    // It captures the real waypoint list and pose plan produced for a block,
    // then compares the fold score of that plan with a plan seeded from the
    // relaxed standby pose.
    internal static class Program28
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

        static double FoldGap(RobotArmModel robot)
        {
            var fk = robot.ForwardKinematics();
            var o = fk.JointOrigins;
            double d = double.MaxValue;
            d = Math.Min(d, SegmentDistance(o[1], o[2], o[3], o[4]));
            d = Math.Min(d, SegmentDistance(o[2], o[3], o[4], fk.EePos));
            d = Math.Min(d, SegmentDistance(o[1], o[2], o[4], fk.EePos));
            return d;
        }

        // Worst fold along the joint-interpolated plan, the same way the arm
        // actually moves between plan entries.
        static double PlanFoldGap(List<double[]> plan, out string where)
        {
            var robot = new RobotArmModel();
            double worst = double.MaxValue;
            where = string.Empty;
            for (int i = 0; i < plan.Count; i++)
            {
                Array.Copy(plan[i], robot.Angles, plan[i].Length);
                double gap = FoldGap(robot);
                if (gap < worst)
                {
                    worst = gap;
                    where = $"pose {i} [{string.Join(", ", plan[i].Select(a => a.ToString("F1")))}]";
                }

                if (i == 0) continue;
                var prev = plan[i - 1];
                for (int s = 1; s <= 8; s++)
                {
                    double t = s / 8.0;
                    for (int j = 0; j < prev.Length; j++)
                    {
                        double delta = plan[i][j] - prev[j];
                        while (delta > 180) delta -= 360;
                        while (delta < -180) delta += 360;
                        robot.Angles[j] = prev[j] + delta * t;
                    }

                    gap = FoldGap(robot);
                    if (gap < worst)
                    {
                        worst = gap;
                        where = $"between {i - 1}/{i} t={t:F2} [{string.Join(", ", robot.Angles.Select(a => a.ToString("F1")))}]";
                    }
                }
            }

            return worst;
        }

        static List<double[]>? SolveFrom(Point3D[] waypoints, double[] seed)
        {
            var twin = new RobotArmModel();
            Array.Copy(seed, twin.Angles, seed.Length);
            var poses = new List<double[]>(waypoints.Length);
            foreach (var target in waypoints)
            {
                bool ok = false;
                for (int attempt = 0; attempt < 200; attempt++)
                {
                    twin.SolvePosition(target, 14, 1.0);
                    if ((twin.ForwardKinematics().EePos - target).Length < 0.07)
                    {
                        ok = true;
                        break;
                    }
                }

                if (!ok)
                    return null;
                poses.Add((double[])twin.Angles.Clone());
            }

            return poses;
        }

        public static void Run()
        {
            Console.WriteLine("=== TRANSPORT POSE PLAN FOLD ANALYSIS ===");
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

            var planField = typeof(AutomationController).GetField("_posePlan", BindingFlags.NonPublic | BindingFlags.Instance)!;
            var pathField = typeof(AutomationController).GetField("_transportPath", BindingFlags.NonPublic | BindingFlags.Instance)!;

            bool captured = false;
            for (int tick = 0; tick < 60000 && automation.IsRunning && !captured; tick++)
            {
                automation.Tick();
                var path = (List<Point3D>)pathField.GetValue(automation)!;
                var plan = (List<double[]>)planField.GetValue(automation)!;
                if (automation.Phase == AutomationPhase.MoveToDrop && path.Count >= 2)
                {
                    captured = true;
                    Console.WriteLine($"captured at tick {tick}, waypoints {path.Count}, poses {plan.Count}, index {automation.Index}");

                    var waypoints = path.ToArray();
                    for (int i = 0; i < waypoints.Length; i += Math.Max(1, waypoints.Length / 10))
                        Console.WriteLine($"  wp {i}: ({waypoints[i].X:F2},{waypoints[i].Y:F2},{waypoints[i].Z:F2})");

                    double liveGap = PlanFoldGap(plan, out string liveWhere);
                    Console.WriteLine($"live plan fold gap: {liveGap:F3} at {liveWhere}");

                    var standbySeed = (double[])robot.InitialFoldedAngles.Clone();
                    var standbyPlan = SolveFrom(waypoints, standbySeed);
                    if (standbyPlan == null)
                    {
                        Console.WriteLine("standby-seeded plan: solver failed");
                    }
                    else
                    {
                        double standbyGap = PlanFoldGap(standbyPlan, out string standbyWhere);
                        Console.WriteLine($"standby-seeded plan fold gap: {standbyGap:F3} at {standbyWhere}");
                        for (int i = 0; i < standbyPlan.Count; i += Math.Max(1, standbyPlan.Count / 10))
                            Console.WriteLine($"  sp {i} [{string.Join(", ", standbyPlan[i].Select(a => a.ToString("F1")))}]");
                    }
                }
            }
        }
    }
}
