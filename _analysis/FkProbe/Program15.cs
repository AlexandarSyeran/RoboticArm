using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program15
    {
        public static void Run()
        {
            Console.WriteLine("=== MULTI-SEED REPLAY ===");
            foreach (var seedValue in new[] { 7919, 11, 2024, 555, 90125 })
            {
                Replay(seedValue);
            }
        }

        private static void Replay(int seedValue)
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

            var obstacles = ObstacleLayout.Generate(Math.Abs(pickup.X), seedValue);
            var controller = new AutomationController(robot, items, obstacles, pickup, drop, folded.EePos)
            {
                ArrivalDelay = 0.3,
                ActionDelay = 0.3,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            double worstPayload = 0;
            double worstEe = 0;
            string worstPhase = "";
            string worstPayloadPhase = "";
            int worstPayloadObstacle = -1;
            string modes = "";
            controller.Start();

            for (int tick = 0; tick < 80000 && controller.IsRunning; tick++)
            {
                controller.Tick();

                var attached = controller.AttachedItem;
                var ee = robot.ForwardKinematics().EePos;

                foreach (var o in obstacles)
                {
                    if (attached != null)
                    {
                        double pen = Penetration(attached.Position.X, attached.Position.Y, attached.Position.Z, 0.22, o);
                        if (pen > worstPayload)
                        {
                            worstPayload = pen;
                            worstPayloadPhase = $"{controller.Phase} t={tick * 0.03:F2}s idx={controller.PathIndex} item ({attached.Position.X:F2},{attached.Position.Y:F2},{attached.Position.Z:F2}) EE ({ee.X:F2},{ee.Y:F2},{ee.Z:F2}) obstacle ({o.Center.X:F2},{o.Center.Y:F2},{o.Center.Z:F2}) half ({o.HalfSize.X:F2},{o.HalfSize.Y:F2},{o.HalfSize.Z:F2})";
                            worstPayloadObstacle = obstacles.IndexOf(o);
                        }
                    }

                    double eePen = Penetration(ee.X, ee.Y, ee.Z, 0.18, o);
                    if (eePen > worstEe)
                    {
                        worstEe = eePen;
                        worstPhase = $"{controller.Phase} t={tick * 0.03:F2}s EE ({ee.X:F2},{ee.Y:F2},{ee.Z:F2})";
                    }
                }

                if (controller.Phase == AutomationPhase.MoveToDrop && !modes.Contains(controller.Status))
                {
                    if (controller.Status.Contains("("))
                        modes += controller.Status;
                }
            }

            int done = 0;
            foreach (var item in items)
                if (item.Completed) done++;

            var posePlan = (System.Collections.IList)typeof(AutomationController)
                .GetField("_posePlan", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                .GetValue(controller);

            Console.WriteLine($"seed {seedValue,6}: {done}/4 payloadPen {worstPayload:F3} eePen {worstEe:F3} status='{controller.Status}' phase={controller.Phase} pathIndex={controller.PathIndex} poses={posePlan.Count}");
            if (!controller.IsRunning && controller.Status.Contains("stopped"))
            {
                var twin = new RobotArmModel();
                int firstFail = -1;
                for (int i = 0; i < controller.TransportPath.Count; i++)
                {
                    bool ok = false;
                    for (int attempt = 0; attempt < 200 && !ok; attempt++)
                    {
                        twin.SolvePosition(controller.TransportPath[i], 14, 1.0);
                        ok = (twin.ForwardKinematics().EePos - controller.TransportPath[i]).Length < 0.07;
                    }

                    if (!ok)
                    {
                        firstFail = i;
                        var wp = controller.TransportPath[i];
                        Console.WriteLine($"          twin FAIL @ {i} ({wp.X:F2},{wp.Y:F2},{wp.Z:F2}) residual {(twin.ForwardKinematics().EePos - wp).Length:F3}");
                        break;
                    }
                }
                if (firstFail < 0)
                    Console.WriteLine($"          twin ok over all {controller.TransportPath.Count} waypoints from folded");
            }
            if (worstPhase.Length > 0)
                Console.WriteLine($"          worstEE @ {worstPhase}");
            if (worstPayloadPhase.Length > 0)
                Console.WriteLine($"          worstPayload @ {worstPayloadPhase}");
            Console.WriteLine($"          routes: {modes}");
            Console.WriteLine($"          waypoints: {controller.TransportPath.Count}");
            foreach (var p in controller.TransportPath)
                Console.WriteLine($"          ({p.X,5:F2}, {p.Y,5:F2}, {p.Z,5:F2})");
        }

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
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
