using System;
using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace FkProbe
{
    internal static class Program13
    {
        public static void Run()
        {
            Console.WriteLine("=== CONTINUOUS CARRY BAND ===");

            foreach (var y in new[] { 1.0, 1.15, 1.3, 1.45, 1.6, 1.76, 2.0, 2.2, 2.5 })
                Traverse(y);

            foreach (var x in new[] { 3.6, 2.9, 2.2, 1.4, 0.2, -0.9, -2.0, -3.2 })
                Climb(x);
        }

        private static void Traverse(double y)
        {
            var robot = new RobotArmModel();
            var seed = (double[])robot.InitialFoldedAngles.Clone();
            Array.Copy(seed, robot.Angles, seed.Length);
            Converge(robot, new Point3D(3.74, y, -0.35));

            double jam = 0;
            for (double x = 3.6; x >= -4.2; x -= 0.14)
            {
                var target = new Point3D(x, y, Math.Min(-0.35, 0));
                if (!Converge(robot, target))
                {
                    jam = x;
                    break;
                }
            }

            Console.WriteLine($"y={y,5:F2}: traverse {(jam == 0 ? "complete" : $"jams at x={jam,5:F2}")}");
        }

        private static void Climb(double x)
        {
            var robot = new RobotArmModel();
            var seed = (double[])robot.InitialFoldedAngles.Clone();
            Array.Copy(seed, robot.Angles, seed.Length);
            Converge(robot, new Point3D(3.74, 1.0, -0.35));
            Converge(robot, new Point3D(x, 1.0, -0.5));

            double jam = 0;
            for (double y = 1.0; y <= 2.6; y += 0.12)
            {
                if (!Converge(robot, new Point3D(x, y, -0.5)))
                {
                    jam = y;
                    break;
                }
            }

            Console.WriteLine($"x={x,5:F2}: climb {(jam == 0 ? "complete to 2.6" : $"jams at y={jam,5:F2}")}");
        }

        private static bool Converge(RobotArmModel robot, Point3D target)
        {
            for (int attempt = 0; attempt < 200; attempt++)
            {
                robot.SolvePosition(target, 14, 1.0);
                if ((robot.ForwardKinematics().EePos - target).Length < 0.05)
                    return true;
            }

            return false;
        }
    }
}
