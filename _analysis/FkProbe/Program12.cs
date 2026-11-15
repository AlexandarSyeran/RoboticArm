using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program12
    {
        public static void Run()
        {
            Console.WriteLine("=== TRANSPORT IK TRACKING ===");

            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
            var pickup = new Point3D(folded.EePos.X, 0.30, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);

            var obstacles = new List<Obstacle>
            {
                new() { Center = new Point3D(-2.90, 2.40, 0.20), HalfSize = new Vector3D(0.18, 0.50, 1.7) },
                new() { Center = new Point3D(-1.49, 2.40, -0.10), HalfSize = new Vector3D(0.18, 0.50, 1.7) },
                new() { Center = new Point3D(0.02, 0.57, -0.04), HalfSize = new Vector3D(0.18, 0.45, 1.7) },
                new() { Center = new Point3D(1.45, 0.57, 0.20), HalfSize = new Vector3D(0.18, 0.45, 1.7) },
                new() { Center = new Point3D(2.91, 1.80, -0.07), HalfSize = new Vector3D(0.20, 1.60, 0.22) }
            };

            var start = new Point3D(3.74, 0.74, -0.35);
            var end = new Point3D(drop.X, drop.Y + 0.45, drop.Z);
            var offset = new Vector3D(-0.14, -0.32, 0.01);

            foreach (var mid in new[] { 1.76, 2.10, 2.40 })
            {
                var plan = TransportPathPlanner.Plan(start, end, obstacles, offset, 0.24, 1.0, 2.5);
                var points = Densify(plan.Points, 0.25);
                Console.WriteLine($"mid candidate {mid:F2} | plan mode '{plan.Mode}' waypoints {plan.Points.Count} dense {points.Count}");

                var probe = new RobotArmModel();
                double[] seed = { -5, -2, -56, 108, -101 };
                Array.Copy(seed, probe.Angles, seed.Length);

                int failed = -1;
                for (int i = 0; i < points.Count; i++)
                {
                    bool ok = false;
                    for (int attempt = 0; attempt < 100 && !ok; attempt++)
                        ok = probe.SolvePosition(points[i], 14, 0.55).Success;

                    if (!ok)
                    {
                        failed = i;
                        var fk = probe.ForwardKinematics();
                        Console.WriteLine($"    stuck at {i}: target ({points[i].X:F2},{points[i].Y:F2},{points[i].Z:F2}) EE ({fk.EePos.X:F2},{fk.EePos.Y:F2},{fk.EePos.Z:F2}) angles [{string.Join(", ", Array.ConvertAll(probe.Angles, a => a.ToString("F0")))}]");
                        break;
                    }
                }

                Console.WriteLine(failed < 0 ? "    all waypoints tracked" : "    route not trackable");
            }

            var stuckPose = new[] { -22.9, 11.3, -120.0, 150.0, -120.0 };
            foreach (var y in new[] { 1.76, 2.0, 2.1, 2.3, 2.5 })
            {
                var probe = new RobotArmModel();
                Array.Copy(stuckPose, probe.Angles, stuckPose.Length);
                var result = probe.SolvePosition(new Point3D(1.49, y, -0.63), 300, 0.55);
                Console.WriteLine($"from stuck pose -> y={y:F2}: success={result.Success} err={result.RemainingError:F3} angles [{string.Join(", ", Array.ConvertAll(probe.Angles, a => a.ToString("F0")))}]");
            }
        }

        private static List<Point3D> Densify(List<Point3D> points, double maxGap)
        {
            var dense = new List<Point3D> { points[0] };
            for (int i = 1; i < points.Count; i++)
            {
                var from = points[i - 1];
                var to = points[i];
                int steps = Math.Max(1, (int)Math.Ceiling((to - from).Length / maxGap));
                for (int step = 1; step <= steps; step++)
                    dense.Add(from + (to - from) * ((double)step / steps));
            }

            return dense;
        }
    }
}
