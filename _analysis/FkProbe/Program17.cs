using System;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program17
    {
        // Plan-only sanity check of the obstacle course: prints the route the
        // planner picks for the first several tests without simulating ticks.
        public static void Run()
        {
            Console.WriteLine("=== PLAN-ONLY ROUTE DUMP ===");
            var robot = new RobotArmModel();
            double reachX = Math.Abs(robot.ForwardKinematics().EePos.X);
            var start = new Point3D(reachX, 0.46, 0);
            var end = new Point3D(-reachX, 0.75, 0);

            for (int test = 1; test <= 8; test++)
            {
                var obstacles = ObstacleLayout.Generate(reachX, test);
                var offset = new Vector3D(0, -0.5, 0);
                var plan = TransportPathPlanner.Plan(start, end, obstacles, offset, 0.24, 1.0, 4.85, 4.743);

                Console.WriteLine($"test {test}: mode={plan.Mode} points={plan.Points.Count}");
                foreach (var p in plan.Points)
                    Console.WriteLine($"          ({p.X,5:F2}, {p.Y,5:F2}, {p.Z,5:F2})");
            }
        }
    }
}
