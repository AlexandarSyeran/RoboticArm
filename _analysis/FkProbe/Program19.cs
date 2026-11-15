using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program19
    {
        // Investigates the abort on the obstacle course and the wrist (green
        // link) usage hypothesis: dumps the controller's actual route, then
        // replays the twin pose sequence measuring per-joint travel, and
        // finally scans the wrist seed at failing waypoints to see whether
        // the frozen wrist is what blocks convergence.
        public static void Run()
        {
            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
            var pickup = new Point3D(folded.EePos.X, 0.30, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);

            var items = new List<AutomationItem>
            {
                Item(pickup.X - 0.45, 0.46, pickup.Z - 0.35, 0),
                Item(pickup.X + 0.45, 0.46, pickup.Z - 0.35, 1),
                Item(pickup.X - 0.45, 0.46, pickup.Z + 0.35, 2),
                Item(pickup.X + 0.45, 0.46, pickup.Z + 0.35, 3)
            };

            var obstacles = ObstacleLayout.Generate(Math.Abs(pickup.X), 7919);
            var controller = new AutomationController(robot, items, obstacles, pickup, drop, folded.EePos)
            {
                ArrivalDelay = 0.3,
                ActionDelay = 0.3,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            controller.Start();
            var lastPhase = controller.Phase;
            string lastStatus = "";
            int carries = 0;
            for (int tick = 0; tick < 80000 && controller.IsRunning; tick++)
            {
                var phaseBefore = controller.Phase;
                controller.Tick();
                if (phaseBefore != controller.Phase && controller.Phase == AutomationPhase.MoveToDrop)
                {
                    carries++;
                    var modeNow = typeof(AutomationController)
                        .GetField("_routeMode", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                        ?.GetValue(controller);
                    Console.WriteLine($"--- carry {carries}: routeMode={modeNow} pts={controller.TransportPath.Count}");
                    for (int i = 0; i < controller.TransportPath.Count; i += 6)
                    {
                        var p = controller.TransportPath[i];
                        Console.WriteLine($"    wp[{i,2}] ({p.X:F2},{p.Y:F2},{p.Z:F2})");
                    }
                }
                if (carries == 1 && controller.Phase == AutomationPhase.MoveToDrop && tick % 5 == 0)
                {
                    var ee = robot.ForwardKinematics().EePos;
                    Console.WriteLine($"  carry1 t={tick * 0.03:F2}s idx={controller.PathIndex,2} EE ({ee.X:F2},{ee.Y:F2},{ee.Z:F2}) angles [{string.Join(",", Array.ConvertAll(robot.Angles, a => a.ToString("F0")))}]");
                }
                if (controller.Status != lastStatus)
                {
                    Console.WriteLine($"t={tick * 0.03,7:F2}s {controller.Phase,-12} {controller.Status}");
                    lastStatus = controller.Status;
                }
                if (controller.IsRunning)
                    lastPhase = controller.Phase;
            }

            Console.WriteLine($"ABORTED in phase {lastPhase}, pathIndex={controller.PathIndex}, poses={PosePlanCount(controller)}");
            Console.WriteLine($"ToolOffset={controller.ToolOffset}");
            var routeMode = typeof(AutomationController)
                .GetField("_routeMode", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                ?.GetValue(controller);
            Console.WriteLine($"routeMode={routeMode}");
            for (int i = 0; i < controller.TransportPath.Count; i++)
            {
                var p = controller.TransportPath[i];
                Console.WriteLine($"  wp[{i,2}] ({p.X,5:F2},{p.Y,5:F2},{p.Z,5:F2})");
            }

            JointUsage(controller.TransportPath);
        }

        private static int PosePlanCount(AutomationController controller)
        {
            var plan = (System.Collections.IList)typeof(AutomationController)
                .GetField("_posePlan", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .GetValue(controller);
            return plan.Count;
        }

        private static void JointUsage(IReadOnlyList<Point3D> path)
        {
            var names = new[] { "base yaw", "elev", "shoulder", "elbow", "WRIST" };
            var twin = new RobotArmModel();
            var prev = (double[])twin.Angles.Clone();
            var travel = new double[twin.Angles.Length];
            var min = (double[])twin.Angles.Clone();
            var max = (double[])twin.Angles.Clone();
            var failures = new List<int>();

            for (int i = 0; i < path.Count; i++)
            {
                bool ok = false;
                for (int attempt = 0; attempt < 200 && !ok; attempt++)
                {
                    twin.SolvePosition(path[i], 14, 1.0);
                    ok = (twin.ForwardKinematics().EePos - path[i]).Length < 0.07;
                }

                if (!ok)
                {
                    failures.Add(i);
                    continue;
                }

                for (int j = 0; j < travel.Length; j++)
                {
                    travel[j] += Math.Abs(twin.Angles[j] - prev[j]);
                    min[j] = Math.Min(min[j], twin.Angles[j]);
                    max[j] = Math.Max(max[j], twin.Angles[j]);
                    prev[j] = twin.Angles[j];
                }
            }

            Console.WriteLine("joint usage across route (deg of travel / range used):");
            for (int j = 0; j < travel.Length; j++)
                Console.WriteLine($"  {names[j],9}: travel {travel[j],7:F1}  range [{min[j],7:F1}..{max[j],7:F1}]");
            Console.WriteLine($"convergence failures at waypoints: {(failures.Count == 0 ? "none" : string.Join(",", failures))}");

            foreach (int fail in failures)
            {
                var wp = path[fail];
                double best = double.MaxValue;
                double bestSeed = 0;
                for (double seed = -120; seed <= 120; seed += 10)
                {
                    var scan = new RobotArmModel();
                    scan.Angles[4] = seed;
                    for (int attempt = 0; attempt < 200; attempt++)
                        scan.SolvePosition(wp, 14, 1.0);
                    double residual = (scan.ForwardKinematics().EePos - wp).Length;
                    if (residual < best)
                    {
                        best = residual;
                        bestSeed = seed;
                    }
                }

                Console.WriteLine($"  wp[{fail}] ({wp.X:F2},{wp.Y:F2},{wp.Z:F2}): residual with frozen-wrist replay vs wrist-seed scan best {best:F3} @ wrist {bestSeed:F0}");
            }
        }

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
        }
    }
}
