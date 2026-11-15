using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace FkProbe
{
    // Tests whether routing the no-obstacle transport over the base at the
    // clearance height (instead of a low straight line) yields a pose
    // sequence that does not fold the arm into itself.
    internal static class Program29
    {
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

        static double FoldGap(RobotArmModel robot, out string angles)
        {
            var fk = robot.ForwardKinematics();
            var o = fk.JointOrigins;
            double d = double.MaxValue;
            d = Math.Min(d, SegmentDistance(o[1], o[2], o[3], o[4]));
            d = Math.Min(d, SegmentDistance(o[2], o[3], o[4], fk.EePos));
            d = Math.Min(d, SegmentDistance(o[1], o[2], o[4], fk.EePos));
            angles = string.Join(", ", robot.Angles.Select(a => a.ToString("F1")));
            return d;
        }

        static IEnumerable<Point3D> Densify(IReadOnlyList<Point3D> points, double maxGap = 0.25)
        {
            yield return points[0];
            for (int i = 1; i < points.Count; i++)
            {
                var from = points[i - 1];
                var to = points[i];
                int steps = Math.Max(1, (int)Math.Ceiling((to - from).Length / maxGap));
                for (int step = 1; step <= steps; step++)
                    yield return from + (to - from) * ((double)step / steps);
            }
        }

        static bool SolveRoute(List<Point3D> waypoints, double[] seed, out double worstGap, out string worstAngles, out string failAt)
        {
            var robot = new RobotArmModel();
            Array.Copy(seed, robot.Angles, seed.Length);
            worstGap = double.MaxValue;
            worstAngles = string.Empty;
            failAt = string.Empty;
            double[] previous = (double[])robot.Angles.Clone();

            foreach (var target in waypoints)
            {
                bool ok = false;
                for (int attempt = 0; attempt < 200; attempt++)
                {
                    robot.SolvePosition(target, 14, 1.0);
                    if ((robot.ForwardKinematics().EePos - target).Length < 0.07)
                    {
                        ok = true;
                        break;
                    }
                }

                if (!ok)
                {
                    failAt = $"({target.X:F2},{target.Y:F2},{target.Z:F2})";
                    return false;
                }

                double gap = FoldGap(robot, out string a);
                if (gap < worstGap) { worstGap = gap; worstAngles = a; }

                for (int s = 1; s <= 8; s++)
                {
                    double t = s / 8.0;
                    var probe = new RobotArmModel();
                    for (int j = 0; j < previous.Length; j++)
                    {
                        double delta = robot.Angles[j] - previous[j];
                        while (delta > 180) delta -= 360;
                        while (delta < -180) delta += 360;
                        probe.Angles[j] = previous[j] + delta * t;
                    }

                    double ig = FoldGap(probe, out string ia);
                    if (ig < worstGap) { worstGap = ig; worstAngles = ia; }
                }

                previous = (double[])robot.Angles.Clone();
            }

            return true;
        }

        public static void Run()
        {
            Console.WriteLine("=== RAISED CROSSING ROUTE TEST ===");
            var standby = new RobotArmModel();
            var seed = (double[])standby.Angles.Clone();

            var pickup = new Point3D(4.192, 0.30, 0);
            var drop = new Point3D(-4.192, 0.30, 0);
            var pick = new Point3D(3.74, 0.74, -0.35);
            var lift = new Point3D(pick.X, pick.Y + 0.6, pick.Z);
            var dropHover = new Point3D(-4.412, 0.84, -0.22);
            var approach = new Point3D(dropHover.X, dropHover.Y + 0.25, dropHover.Z);

            var variants = new Dictionary<string, List<Point3D>>
            {
                ["current straight"] = new() { lift, approach, dropHover },
                ["raised 2.6"] = new() { lift, new Point3D(0, 2.6, 0), approach, dropHover },
                ["raised 2.2"] = new() { lift, new Point3D(0, 2.2, 0), approach, dropHover },
                ["raised pickup+drop clearance"] = new()
                {
                    lift,
                    new Point3D(0, 2.6, 0),
                    new Point3D(-2.2, Math.Max(2.0, dropHover.Y + 1.0), -0.11),
                    approach,
                    dropHover
                }
            };

            foreach (var pair in variants)
            {
                var dense = Densify(pair.Value).ToList();
                bool ok = SolveRoute(dense, seed, out double gap, out string angles, out string failAt);
                if (!ok)
                {
                    Console.WriteLine($"{pair.Key}: IK failed at {failAt} ({dense.Count} waypoints)");
                    continue;
                }

                Console.WriteLine($"{pair.Key}: {dense.Count} waypoints, worst fold gap {gap:F3} [{angles}]");
            }
        }
    }
}
