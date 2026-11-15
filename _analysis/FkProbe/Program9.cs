using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program9
    {
        private static string Fmt(Point3D p) => $"({p.X,7:F2},{p.Y,6:F2},{p.Z,7:F2})";

        public static void Run()
        {
            Console.WriteLine("=== OBSTACLE TEST REPLAY (NEW LAYOUT + PLANNER) ===");

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

            var obstacles = BuildLayout(pickup, drop);
            Console.WriteLine($"obstacles: {obstacles.Count}");
            foreach (var o in obstacles)
                Console.WriteLine($"  type h={o.HalfSize.Y:F2} center {Fmt(o.Center)} y[{o.Center.Y - o.HalfSize.Y:F2}..{o.Center.Y + o.HalfSize.Y:F2}]");

            var controller = new AutomationController(robot, items, obstacles, pickup, drop, folded.EePos)
            {
                ArrivalDelay = 0.3,
                ActionDelay = 0.3,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            string last = null;
            int phaseTicks = 0;
            int modePrints = 0;
            int lastIndex = -1;
            double worstPayload = 0;
            double worstEe = 0;
            controller.Start();

            for (int tick = 0; tick < 80000 && controller.IsRunning; tick++)
            {
                var before = controller.Phase;
                controller.Tick();
                phaseTicks++;

                if (!controller.IsRunning && controller.Status.Contains("stopped"))
                {
                    var fk = robot.ForwardKinematics();
                    var target = controller.PathIndex < controller.TransportPath.Count
                        ? controller.TransportPath[controller.PathIndex]
                        : default;
                    Console.WriteLine($"    abort at pathIndex={controller.PathIndex} target {Fmt(target)} angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F1")))}] EE {Fmt(fk.EePos)}");
                }

                var attached = controller.AttachedItem;
                if (attached != null)
                {
                    foreach (var o in obstacles)
                    {
                        double pen = Penetration(attached.Position.X, attached.Position.Y, attached.Position.Z, 0.22, o);
                        if (pen > 0.08 && pen > worstPayload)
                        {
                            var ee = robot.ForwardKinematics().EePos;
                            Console.WriteLine($"    CLIP t={tick * 0.03:F2}s idx={controller.Index} {controller.Phase} pathIndex={controller.PathIndex} pen={pen:F2} item {Fmt(attached.Position)} EE {Fmt(ee)} vs obstacle {Fmt(o.Center)}");
                        }

                        worstPayload = Math.Max(worstPayload, pen);
                        var eePos = robot.ForwardKinematics().EePos;
                        worstEe = Math.Max(worstEe, Penetration(eePos.X, eePos.Y, eePos.Z, 0.18, o));
                    }
                }

                if (controller.Status != last)
                {
                    Console.WriteLine($"t={tick * 0.03,7:F2}s tick={tick,6} idx={controller.Index} {controller.Phase,-12} {controller.Status}");
                    last = controller.Status;
                    phaseTicks = 0;
                    if (controller.Status.Contains("("))
                        modePrints++;
                }

                if (before != controller.Phase && controller.Phase == AutomationPhase.ReturnHome)
                {
                    var posePlan = (System.Collections.IList)typeof(AutomationController)
                        .GetField("_posePlan", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance)
                        .GetValue(controller);
                    Console.WriteLine($"    return leg: points={controller.TransportPath.Count} poses={posePlan.Count} pathIndex={controller.PathIndex} EE {Fmt(robot.ForwardKinematics().EePos)}");

                    if (posePlan.Count == 0)
                    {
                        var twin = new RobotArmModel();
                        Array.Copy(robot.Angles, twin.Angles, robot.Angles.Length);
                        for (int i = 0; i < controller.TransportPath.Count; i++)
                        {
                            bool ok = false;
                            for (int attempt = 0; attempt < 80 && !ok; attempt++)
                            {
                                twin.SolvePosition(controller.TransportPath[i], 14, 1.0);
                                ok = (twin.ForwardKinematics().EePos - controller.TransportPath[i]).Length < 0.05;
                            }

                            if (!ok)
                            {
                                Console.WriteLine($"    return pose fail at {i} {Fmt(controller.TransportPath[i])} EE {Fmt(twin.ForwardKinematics().EePos)} angles [{string.Join(", ", Array.ConvertAll(twin.Angles, a => a.ToString("F1")))}]");
                                break;
                            }
                        }
                    }
                }

                if (before != controller.Phase && controller.Phase == AutomationPhase.MoveToDrop)
                {
                    var fk = robot.ForwardKinematics();
                    var world = robot.ToolOrientation(fk).Transform(controller.ToolOffset);
                    Console.WriteLine($"    drop leg start: angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F0")))}] EE {Fmt(fk.EePos)} toolOffset ({controller.ToolOffset.X:F2},{controller.ToolOffset.Y:F2},{controller.ToolOffset.Z:F2}) world ({world.X:F2},{world.Y:F2},{world.Z:F2})");
                    Console.WriteLine($"    path: {string.Join(" -> ", System.Linq.Enumerable.Select(controller.TransportPath, p => Fmt(p)))}");
                }

                if (controller.Phase == AutomationPhase.MoveToDrop && lastIndex != controller.PathIndex)
                {
                    lastIndex = controller.PathIndex;
                    var fk = robot.ForwardKinematics();
                    var target = controller.PathIndex < controller.TransportPath.Count ? controller.TransportPath[controller.PathIndex] : default;
                    Console.WriteLine($"      wp {controller.PathIndex,2} target {Fmt(target)} EE {Fmt(fk.EePos)} angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F0")))}]");
                }

                if (phaseTicks == 4000)
                {
                    Console.WriteLine($"STALL phase={controller.Phase} status='{controller.Status}' EE {Fmt(robot.ForwardKinematics().EePos)}");
                    break;
                }
            }

            Console.WriteLine($"finished: running={controller.IsRunning} status='{controller.Status}' diagnostic='{controller.Diagnostic}' pathIndex={controller.PathIndex} ticks={modePrints}");
            Console.WriteLine($"worst payload penetration {worstPayload:F3} | worst EE penetration {worstEe:F3}");
            int done = 0;
            foreach (var item in items)
            {
                if (item.Completed) done++;
                Console.WriteLine($"  item {item.PaletteIndex}: completed={item.Completed} pos {Fmt(item.Position)}");
            }
            Console.WriteLine($"completed items: {done}/{items.Count}");
        }

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
        }

        // Mirrors MainViewModel.GenerateObstacleLayout for the first test run.
        private static List<Obstacle> BuildLayout(Point3D pickup, Point3D drop)
        {
            var random = new Random(7919);
            var obstacles = new List<Obstacle>();
            var forbidden = new[] { pickup, drop };
            var types = new int[6];
            types[0] = types[5] = 1;
            types[1] = types[4] = 2;
            types[2] = random.Next(2) == 0 ? 0 : 2;
            types[3] = random.Next(2) == 0 ? 0 : 2;

            const double spacing = 1.2;
            double first = -spacing * (types.Length - 1) * 0.5;

            for (int i = 0; i < types.Length; i++)
            {
                Point3D center = default;
                var halfSize = new Vector3D(0.18, 0.3, 0.18);
                double band = first + i * spacing;

                for (int attempt = 0; attempt < 100; attempt++)
                {
                    double x = band + (random.NextDouble() - 0.5) * 0.16;
                    double z = (random.NextDouble() - 0.5) * 1.1;

                    switch (types[i])
                    {
                        case 0:
                            halfSize = new Vector3D(0.18, 0.45, 1.7);
                            center = new Point3D(x, 0.12 + halfSize.Y, z * 0.4);
                            break;
                        case 1:
                            halfSize = new Vector3D(0.18, 0.5, 1.7);
                            center = new Point3D(x, 2.4, z * 0.4);
                            break;
                        default:
                            halfSize = new Vector3D(0.2, 1.6, 0.22);
                            center = new Point3D(x, 0.2 + halfSize.Y, z * 0.5);
                            break;
                    }

                    if (!IsNearZone(center, halfSize, forbidden[0], 0.75) &&
                        !IsNearZone(center, halfSize, forbidden[1], 0.75) &&
                        !Overlaps(center, halfSize, obstacles))
                    {
                        break;
                    }
                }

                obstacles.Add(new Obstacle { Center = center, HalfSize = halfSize });
            }

            return obstacles;
        }

        private static double Penetration(double x, double y, double z, double half, Obstacle o)
        {
            double ox = Math.Min(x + half, o.Center.X + o.HalfSize.X) - Math.Max(x - half, o.Center.X - o.HalfSize.X);
            double oy = Math.Min(y + half, o.Center.Y + o.HalfSize.Y) - Math.Max(y - half, o.Center.Y - o.HalfSize.Y);
            double oz = Math.Min(z + half, o.Center.Z + o.HalfSize.Z) - Math.Max(z - half, o.Center.Z - o.HalfSize.Z);
            return ox > 0 && oy > 0 && oz > 0 ? Math.Min(ox, Math.Min(oy, oz)) : 0;
        }

        private static bool IsNearZone(Point3D center, Vector3D halfSize, Point3D zone, double margin)
            => Math.Abs(center.X - zone.X) < halfSize.X + margin &&
               Math.Abs(center.Z - zone.Z) < halfSize.Z + margin;

        private static bool Overlaps(Point3D center, Vector3D halfSize, List<Obstacle> existing)
        {
            foreach (var obstacle in existing)
            {
                if (Math.Abs(center.X - obstacle.Center.X) < halfSize.X + obstacle.HalfSize.X + 0.15 &&
                    Math.Abs(center.Z - obstacle.Center.Z) < halfSize.Z + obstacle.HalfSize.Z + 0.15)
                {
                    return true;
                }
            }

            return false;
        }
    }
}
