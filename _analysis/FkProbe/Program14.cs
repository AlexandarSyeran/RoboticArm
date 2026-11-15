using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program14
    {
        private static string Fmt(Point3D p) => $"({p.X,6:F2},{p.Y,5:F2},{p.Z,6:F2})";

        public static void Run()
        {
            Console.WriteLine("=== CROSSING FEASIBILITY ===");

            var obstacles = new List<Obstacle>
            {
                new() { Center = new Point3D(-3.07, 2.40, 0.14), HalfSize = new Vector3D(0.18, 0.5, 1.7) },
                new() { Center = new Point3D(-1.80, 1.80, 0.25), HalfSize = new Vector3D(0.2, 1.6, 0.22) },
                new() { Center = new Point3D(-0.67, 1.80, -0.12), HalfSize = new Vector3D(0.2, 1.6, 0.22) },
                new() { Center = new Point3D(0.63, 0.57, -0.04), HalfSize = new Vector3D(0.18, 0.45, 1.7) },
                new() { Center = new Point3D(1.79, 1.80, 0.24), HalfSize = new Vector3D(0.2, 1.6, 0.22) },
                new() { Center = new Point3D(3.01, 2.40, -0.06), HalfSize = new Vector3D(0.18, 0.5, 1.7) }
            };

            var start = new Point3D(3.74, 0.74, -0.35);
            var end = new Point3D(-4.19, 0.75, 0);
            var offset = new Vector3D(-0.14, -0.32, 0.01);
            var seed = new[] { -5.0, -2.0, -56.0, 108.0, -101.0 };

            foreach (var high in new[] { 2.52, 3.95 })
            {
                var plan = TransportPathPlanner.Plan(start, end, obstacles, offset, 0.24, 1.0, high, 4.743);
                var dense = Densify(plan.Points);
                var twin = new RobotArmModel();
                Array.Copy(seed, twin.Angles, seed.Length);

                int failAt = -1;
                for (int i = 0; i < dense.Count; i++)
                {
                    if (!Converge(twin, dense[i]))
                    {
                        failAt = i;
                        break;
                    }
                }

                Console.WriteLine($"high={high:F2} mode={plan.Mode} waypoints={dense.Count} failAt={failAt}");
                if (failAt >= 0)
                {
                    var p = dense[failAt];
                    var prev = failAt > 0 ? dense[failAt - 1] : start;
                    Console.WriteLine($"  fail target {Fmt(p)} prev {Fmt(prev)} r={Math.Sqrt(p.X * p.X + p.Z * p.Z):F2}");
                    Console.WriteLine($"  stuck EE {Fmt(twin.ForwardKinematics().EePos)} angles [{string.Join(", ", Array.ConvertAll(twin.Angles, a => a.ToString("F1")))}]");
                }
            }

            // Direct feasibility of points above the base at the failing height.
            Console.WriteLine("above-base points at y=2.48:");
            foreach (var (x, z) in new[] { (0.10, -0.35), (0.10, -0.77), (0.10, -1.10), (0.35, -0.77), (0.10, -1.40) })
            {
                var target = new Point3D(x, 2.48, z);
                bool ok = false;
                var best = new RobotArmModel();
                Array.Copy(seed, best.Angles, seed.Length);
                for (int i = 0; i < 200 && !ok; i++)
                {
                    best.SolvePosition(target, 20, 1.0);
                    ok = (best.ForwardKinematics().EePos - target).Length < 0.05;
                }
                Console.WriteLine($"  ({x:F2},{z:F2}) r={Math.Sqrt(x * x + z * z):F2} ok={ok} angles [{string.Join(", ", Array.ConvertAll(best.Angles, a => a.ToString("F1")))}] EE {Fmt(best.ForwardKinematics().EePos)}");
            }
        }

        private static bool Converge(RobotArmModel twin, Point3D target)
        {
            for (int attempt = 0; attempt < 80; attempt++)
            {
                twin.SolvePosition(target, 14, 1.0);
                if ((twin.ForwardKinematics().EePos - target).Length < 0.05)
                    return true;
            }

            return false;
        }

        private static List<Point3D> Densify(IReadOnlyList<Point3D> points)
        {
            const double maxGap = 0.25;
            var result = new List<Point3D> { points[0] };
            for (int i = 1; i < points.Count; i++)
            {
                var from = points[i - 1];
                var to = points[i];
                int steps = Math.Max(1, (int)Math.Ceiling((to - from).Length / maxGap));
                for (int step = 1; step <= steps; step++)
                    result.Add(from + (to - from) * ((double)step / steps));
            }
            return result;
        }
    }
}
