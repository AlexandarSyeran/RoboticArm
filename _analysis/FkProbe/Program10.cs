using System;
using System.Collections.Generic;
using System.Reflection;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program10
    {
        public static void Run()
        {
            Console.WriteLine("=== PLANNER SUBSET DIAGNOSTICS ===");
            var start = new Point3D(3.74, 0.74, -0.35);
            var end = new Point3D(-4.19, 0.75, 0.0);
            var payload = new Vector3D(0.0, -0.35, 0.0);

            var wall = new Obstacle { Center = new Point3D(2.0, 0.57, 0.1), HalfSize = new Vector3D(0.18, 0.45, 1.7) };
            var ceiling = new Obstacle { Center = new Point3D(0.8, 2.40, -0.1), HalfSize = new Vector3D(0.18, 0.50, 1.7) };
            var pillar = new Obstacle { Center = new Point3D(-0.4, 1.80, 0.1), HalfSize = new Vector3D(0.20, 1.60, 0.22) };
            var ceiling2 = new Obstacle { Center = new Point3D(-1.6, 2.40, 0.0), HalfSize = new Vector3D(0.18, 0.50, 1.7) };
            var wall2 = new Obstacle { Center = new Point3D(-2.4, 0.57, -0.1), HalfSize = new Vector3D(0.18, 0.45, 1.7) };

            Check("wall+ceiling", start, end, payload, wall, ceiling);
            Check("all", start, end, payload, wall, ceiling, pillar, ceiling2, wall2);

            var isClear = typeof(TransportPathPlanner).GetMethod("IsPoseClear", BindingFlags.NonPublic | BindingFlags.Static);
            var obs = new[] { wall, ceiling };
            Console.WriteLine("node clearance for wall+ceiling (x, y, z):");
            foreach (var x in new[] { 3.74, 2.90, 2.62, 2.34, 2.06, 1.78, 1.50, 1.22, 0.94, 0.66 })
            foreach (var y in new[] { 1.0, 2.135, 3.27 })
            {
                bool clear = (bool)isClear.Invoke(null, new object[] { x, y, -0.35, obs, payload, 0.28, 4.743 });
                Console.WriteLine($"  x={x,5:F2} y={y,5:F3} clear={clear}");
            }
        }

        private static void Check(string name, Point3D start, Point3D end, Vector3D payload, params Obstacle[] obstacles)
        {
            var plan = TransportPathPlanner.Plan(start, end, obstacles, payload, 0.28, 1.0, 3.27);
            Console.WriteLine($"{name}: mode '{plan.Mode}' points {plan.Points.Count}");
            foreach (var p in plan.Points)
                Console.WriteLine($"    ({p.X,6:F2},{p.Y,6:F2},{p.Z,6:F2})");
        }
    }
}
