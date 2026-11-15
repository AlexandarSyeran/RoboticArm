using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program18
    {
        // Diagnoses the transport plan per test: was a grid route found, does
        // the twin IK converge along every dense waypoint, and how much
        // clearance do the EE and payload boxes keep from each obstacle.
        public static void Run()
        {
            Console.WriteLine("=== PLAN DIAGNOSTICS ===");
            var robot = new RobotArmModel();
            double reachX = Math.Abs(robot.ForwardKinematics().EePos.X);
            var start = new Point3D(reachX - 0.45, 0.46, -0.35);
            var end = new Point3D(-reachX, 0.75, 0);
            var offset = new Vector3D(-0.326, -0.117, -0.0007);

            foreach (int test in new[] { 7919, 11, 2024, 555, 90125, 1, 2, 3 })
            {
                var obstacles = ObstacleLayout.Generate(reachX, test);
                if (test > 100) Console.WriteLine();
                var plan = TransportPathPlanner.Plan(start, end, obstacles, offset, 0.24, 1.0, 3.75, 4.743);

                string status = plan.Mode == "unreachable" ? "UNREACHABLE" : plan.Mode;
                foreach (var p in plan.Points) Console.WriteLine($"          raw ({p.X,5:F2},{p.Y,5:F2},{p.Z,5:F2})");
                var dense = Densify(plan.Points);

                var twin = new RobotArmModel();
                Array.Copy(robot.Angles, twin.Angles, robot.Angles.Length);
                string ik = "ok";
                var prev = (double[])twin.Angles.Clone();
                var travel = new double[twin.Angles.Length];
                foreach (var wp in dense)
                {
                    double residual = Converge(twin, wp);
                    if (residual > 0.07)
                    {
                        ik = $"FAIL @ ({wp.X:F2},{wp.Y:F2},{wp.Z:F2}) residual {residual:F3}";
                        break;
                    }

                    for (int j = 0; j < travel.Length; j++)
                    {
                        travel[j] += Math.Abs(twin.Angles[j] - prev[j]);
                        prev[j] = twin.Angles[j];
                    }
                }

                string usage = $"travel yaw {travel[0]:F0} / elev {travel[1]:F0} / sh {travel[2]:F0} / elb {travel[3]:F0} / wrist {travel[4]:F0}";

                double minEe = double.MaxValue, minPayload = double.MaxValue;
                foreach (var wp in dense)
                {
                    foreach (var o in obstacles)
                    {
                        minEe = Math.Min(minEe, BoxGap(wp.X, wp.Y, wp.Z, 0.18, o));
                        minPayload = Math.Min(minPayload, BoxGap(wp.X + offset.X, wp.Y + offset.Y, wp.Z + offset.Z, 0.24, o));
                    }
                }

                Console.WriteLine($"test {test,2}: {status,-18} pts {dense.Count,3}  ik {ik}  {usage}  eeGap {minEe:F2} payloadGap {minPayload:F2}");
            }
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

        private static double Converge(RobotArmModel twin, Point3D target)
        {
            for (int attempt = 0; attempt < 200; attempt++)
            {
                twin.SolvePosition(target, 14, 1.0);
                double residual = (twin.ForwardKinematics().EePos - target).Length;
                if (residual < 0.07)
                    return residual;
            }

            return (twin.ForwardKinematics().EePos - target).Length;
        }

        // Distance from an axis-aligned box to the obstacle box; negative =
        // penetration depth.
        private static double BoxGap(double x, double y, double z, double half, Obstacle o)
        {
            double gx = Math.Max(Math.Abs(x - o.Center.X) - o.HalfSize.X - half, 0);
            double gy = Math.Max(Math.Abs(y - o.Center.Y) - o.HalfSize.Y - half, 0);
            double gz = Math.Max(Math.Abs(z - o.Center.Z) - o.HalfSize.Z - half, 0);
            if (gx == 0 && gy == 0 && gz == 0)
                return -Math.Min(
                    Math.Min(o.HalfSize.X + half - Math.Abs(x - o.Center.X), o.HalfSize.Y + half - Math.Abs(y - o.Center.Y)),
                    o.HalfSize.Z + half - Math.Abs(z - o.Center.Z));
            return Math.Sqrt(gx * gx + gy * gy + gz * gz);
        }
    }
}
