using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace FkProbe
{
    internal static class Program11
    {
        public static void Run()
        {
            Console.WriteLine("=== CARRY LEVEL REACHABILITY (strict) ===");

            var seeds = BuildSeeds();
            Console.Write("h\\r ");
            double[] radii = { 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5 };
            foreach (var r in radii)
                Console.Write($"{r,6:F1}");

            Console.WriteLine();

            foreach (var h in new[] { 0.7, 1.0, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2, 2.4 })
            {
                Console.Write($"{h,4:F1} ");
                foreach (var r in radii)
                {
                    double best = 9;
                    foreach (var seed in seeds)
                    {
                        var probe = new RobotArmModel();
                        Array.Copy(seed, probe.Angles, seed.Length);
                        for (int attempt = 0; attempt < 30; attempt++)
                            probe.SolvePosition(new Point3D(r, 0.3 + h, 0), 40, 0.6);

                        best = Math.Min(best, (probe.ForwardKinematics().EePos - new Point3D(r, 0.3 + h, 0)).Length);
                    }

                    Console.Write(best < 0.05 ? "    ok" : $"{best,6:F2}");
                }

                Console.WriteLine();
            }
        }

        private static List<double[]> BuildSeeds()
        {
            var template = new RobotArmModel();
            var seeds = new List<double[]>();
            foreach (var elev in new[] { -30.0, 0.0, 30.0 })
            foreach (var shoulder in new[] { -80.0, -20.0, 40.0 })
            foreach (var elbow in new[] { -100.0, 0.0, 100.0 })
            {
                var angles = (double[])template.InitialFoldedAngles.Clone();
                angles[1] = elev;
                angles[2] = shoulder;
                angles[3] = elbow;
                seeds.Add(angles);
            }

            return seeds;
        }
    }
}
