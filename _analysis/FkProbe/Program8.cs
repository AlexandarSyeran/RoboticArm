using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program8
    {
        private static string Fmt(Point3D p) => $"({p.X,6:F2},{p.Y,5:F2},{p.Z,6:F2})";

        public static void Run()
        {
            Console.WriteLine("=== TRANSPORT PATH PLANNER ===");

            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
            var pickup = new Point3D(folded.EePos.X, 0.30, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);

            var start = new Point3D(pickup.X - 0.45, 0.74, pickup.Z - 0.35);
            var end = new Point3D(drop.X, drop.Y + 0.45, drop.Z);
            var payload = new Vector3D(0.0, -0.35, 0.0);

            Scenario("floor wall", new[]
            {
                new Obstacle { Center = new Point3D(0.3, 0.57, 0.1), HalfSize = new Vector3D(0.18, 0.45, 1.7) }
            }, start, end, payload);

            Scenario("hanging ceiling", new[]
            {
                new Obstacle { Center = new Point3D(0.3, 2.40, 0.1), HalfSize = new Vector3D(0.18, 0.50, 1.7) }
            }, start, end, payload);

            Scenario("tall pillar", new[]
            {
                new Obstacle { Center = new Point3D(0.3, 1.80, 0.0), HalfSize = new Vector3D(0.20, 1.60, 0.22) }
            }, start, end, payload);

            Scenario("mixed layout", new[]
            {
                new Obstacle { Center = new Point3D(2.2, 0.57, 0.1), HalfSize = new Vector3D(0.18, 0.45, 1.7) },
                new Obstacle { Center = new Point3D(0.9, 2.40, -0.1), HalfSize = new Vector3D(0.18, 0.50, 1.7) },
                new Obstacle { Center = new Point3D(-0.4, 1.80, 0.1), HalfSize = new Vector3D(0.20, 1.60, 0.22) },
                new Obstacle { Center = new Point3D(-1.7, 2.40, 0.0), HalfSize = new Vector3D(0.18, 0.50, 1.7) },
                new Obstacle { Center = new Point3D(-3.0, 0.57, -0.1), HalfSize = new Vector3D(0.18, 0.45, 1.7) }
            }, start, end, payload);
        }

        private static void Scenario(string name, IReadOnlyList<Obstacle> obstacles, Point3D start, Point3D end, Vector3D payload)
        {
            var plan = TransportPathPlanner.Plan(start, end, obstacles, payload, 0.24, 1.0, 3.27);
            Console.WriteLine($"{name}: mode '{plan.Mode}', waypoints {plan.Points.Count}");
            foreach (var p in plan.Points)
                Console.WriteLine($"    {Fmt(p)}");

            double worst = WorstPenetration(plan.Points, obstacles, payload, 0.24);
            Console.WriteLine($"    worst penetration: {worst:F3} (0 = clear)");
        }

        // Checks the straight segments between waypoints against the obstacle boxes.
        private static double WorstPenetration(List<Point3D> points, IReadOnlyList<Obstacle> obstacles,
            Vector3D payload, double payloadHalf)
        {
            double worst = 0;
            for (int i = 0; i < points.Count - 1; i++)
            {
                var a = points[i];
                var b = points[i + 1];
                double length = (b - a).Length;
                int steps = Math.Max(1, (int)(length / 0.05));
                for (int s = 0; s <= steps; s++)
                {
                    var p = a + (b - a) * ((double)s / steps);
                    foreach (var o in obstacles)
                    {
                        worst = Math.Max(worst, Penetration(p.X, p.Y, p.Z, 0.18, o));
                        worst = Math.Max(worst, Penetration(
                            p.X + payload.X, p.Y + payload.Y, p.Z + payload.Z, payloadHalf, o));
                    }
                }
            }

            return worst;
        }

        private static double Penetration(double x, double y, double z, double half, Obstacle o)
        {
            double ox = Math.Min(x + half, o.Center.X + o.HalfSize.X) - Math.Max(x - half, o.Center.X - o.HalfSize.X);
            double oy = Math.Min(y + half, o.Center.Y + o.HalfSize.Y) - Math.Max(y - half, o.Center.Y - o.HalfSize.Y);
            double oz = Math.Min(z + half, o.Center.Z + o.HalfSize.Z) - Math.Max(z - half, o.Center.Z - o.HalfSize.Z);
            return ox > 0 && oy > 0 && oz > 0 ? Math.Min(ox, Math.Min(oy, oz)) : 0;
        }
    }
}
