using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace FkProbe
{
    // Compares transport route shapes between the pickup and drop zones:
    // through-the-base straight lines versus a yaw-only arc that keeps the
    // arm extended around the base. Reports IK feasibility and the closest
    // approach between non-adjacent links (a self-fold indicator).
    internal static class Program30
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
            Console.WriteLine("=== TRANSPORT ROUTE SHAPE TEST ===");
            var standby = new RobotArmModel();
            var seed = (double[])standby.Angles.Clone();

            var lift = new Point3D(3.74, 1.34, -0.35);
            var dropHover = new Point3D(-4.412, 0.84, -0.22);
            var approach = new Point3D(dropHover.X, dropHover.Y + 0.25, dropHover.Z);

            List<Point3D> Arc(double radius, double y, int steps)
            {
                var points = new List<Point3D> { lift };
                double start = Math.Atan2(lift.Z, lift.X);
                double end = Math.Atan2(approach.Z, approach.X);
                double sweep = end - start;
                while (sweep < -Math.PI) sweep += 2 * Math.PI;
                while (sweep > Math.PI) sweep -= 2 * Math.PI;
                for (int i = 1; i <= steps; i++)
                {
                    double angle = start + sweep * i / (steps + 1);
                    points.Add(new Point3D(radius * Math.Cos(angle), y, radius * Math.Sin(angle)));
                }

                points.Add(approach);
                points.Add(dropHover);
                return points;
            }

            var variants = new List<(string Name, List<Point3D> Points)>
            {
                ("through base (current)", new List<Point3D> { lift, approach, dropHover }),
                ("arc r3.2 y1.2", Arc(3.2, 1.2, 6)),
                ("arc r3.6 y1.3", Arc(3.6, 1.3, 6)),
                ("arc r2.8 y1.0", Arc(2.8, 1.0, 6)),
                ("arc r4.0 y1.1", Arc(4.0, 1.1, 8)),
                ("arc r4.4 y1.2", Arc(4.4, 1.2, 8))
            };

            foreach (var variant in variants)
            {
                var dense = Densify(variant.Points).ToList();
                bool ok = SolveRoute(dense, seed, out double gap, out string angles, out string failAt);
                if (!ok)
                {
                    Console.WriteLine($"{variant.Name}: IK failed at {failAt} ({dense.Count} waypoints)");
                    continue;
                }

                Console.WriteLine($"{variant.Name}: {dense.Count} waypoints, worst fold gap {gap:F3} [{angles}]");
            }
        }
    }
}
