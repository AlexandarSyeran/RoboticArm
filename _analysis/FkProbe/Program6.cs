using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program6
    {
        public static void Run()
        {
            Console.WriteLine("=== OBSTACLE TEST AUTOMATION REPLAY ===");

            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
            var home = folded.EePos;
            var pickup = new Point3D(folded.EePos.X, 0.30, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);

            var elevUp = (double[])robot.InitialFoldedAngles.Clone();
            elevUp[1] = 45;
            var elevDown = (double[])robot.InitialFoldedAngles.Clone();
            elevDown[1] = -45;
            Console.WriteLine($"elev +45 EE {Fmt(robot.ForwardKinematics(elevUp).EePos)} | elev -45 EE {Fmt(robot.ForwardKinematics(elevDown).EePos)} | folded {Fmt(home)}");

            Console.WriteLine($"folded EE {Fmt(home)}  pickup {Fmt(pickup)}  drop {Fmt(drop)}");

            var items = new List<AutomationItem>
            {
                NewItem(pickup.X - 0.45, 0.46, pickup.Z - 0.35, 0),
                NewItem(pickup.X + 0.45, 0.46, pickup.Z - 0.35, 1),
                NewItem(pickup.X - 0.45, 0.46, pickup.Z + 0.35, 2),
                NewItem(pickup.X + 0.45, 0.46, pickup.Z + 0.35, 3)
            };

            var obstacles = BuildObstacles(pickup, drop);
            Console.WriteLine($"obstacles: {obstacles.Count}");
            foreach (var o in obstacles)
                Console.WriteLine($"  center {Fmt(o.Center)} top {o.Center.Y + o.HalfSize.Y:0.00}");

            var controller = new AutomationController(robot, items, obstacles, pickup, drop, home)
            {
                ArrivalDelay = 2.0,
                ActionDelay = 0.7,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            string lastStatus = null;
            var phaseTicks = 0;
            controller.Start();

            for (int tick = 0; tick < 40000; tick++)
            {
                var phaseBefore = controller.Phase;
                controller.Tick();
                phaseTicks++;

                if (controller.Status != lastStatus)
                {
                    Console.WriteLine($"t={tick * 0.03,7:F2}s tick={tick,6} idx={controller.Index} phase={controller.Phase,-13} {controller.Status}");
                    lastStatus = controller.Status;
                    phaseTicks = 0;
                }

                if (phaseBefore != controller.Phase)
                {
                    var fk = robot.ForwardKinematics();
                    Console.WriteLine($"    -> angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F1")))}] EE {Fmt(fk.EePos)} pickTarget {Fmt(controller.PickTarget)}");
                    phaseTicks = 0;
                }

                if (controller.Phase == AutomationPhase.Idle && controller.Status.StartsWith("Operation finished"))
                    break;

                if (phaseTicks == 3000)
                {
                    var fk = robot.ForwardKinematics();
                    Console.WriteLine($"STALL at idx={controller.Index} phase={controller.Phase} status=\"{controller.Status}\"");
                    Console.WriteLine($"  angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F2")))}]");
                    Console.WriteLine($"  EE {Fmt(fk.EePos)}  pickTarget {Fmt(controller.PickTarget)}");
                    Console.WriteLine($"  attached {(controller.AttachedItem != null ? controller.AttachedItem.PaletteIndex.ToString() : "none")}  itemPos {(controller.AttachedItem != null ? Fmt(controller.AttachedItem.Position) : "-")}");
                    Console.WriteLine($"  item1 completed={items[0].Completed}  item2 completed={items[1].Completed}");
                    break;
                }
            }

            Console.WriteLine("final phases/status: " + controller.Status);
            foreach (var item in items)
                Console.WriteLine($"item {item.PaletteIndex} completed={item.Completed} pos {Fmt(item.Position)}");

            Console.WriteLine("=== RECOVERY PROBE ===");
            var stuck = new RobotArmModel();
            double[] stuckAngles = { -96.89, 23.40, -120.00, 44.89, -112.96 };
            Array.Copy(stuckAngles, stuck.Angles, stuckAngles.Length);
            var fkStuck = stuck.ForwardKinematics();
            Console.WriteLine($"start EE {Fmt(fkStuck.EePos)} home {Fmt(home)} err {(fkStuck.EePos - home).Length:F3}");
            for (int attempt = 0; attempt < 5; attempt++)
            {
                var res = stuck.SolvePosition(home, 14, 0.55);
                var fk = stuck.ForwardKinematics();
                Console.WriteLine($"attempt {attempt}: success={res.Success} remaining={res.RemainingError:F3} EE {Fmt(fk.EePos)} angles [{string.Join(", ", Array.ConvertAll(stuck.Angles, a => a.ToString("F2")))}]");
            }

            Console.WriteLine("-- best single-joint improvements from stuck pose --");
            var basePose = new RobotArmModel();
            Array.Copy(stuckAngles, basePose.Angles, stuckAngles.Length);
            double baseErr = (basePose.ForwardKinematics().EePos - home).Length;
            for (int joint = 0; joint < stuckAngles.Length; joint++)
            {
                var probe = new RobotArmModel();
                Array.Copy(stuckAngles, probe.Angles, stuckAngles.Length);
                double best = baseErr;
                double bestDelta = 0;
                foreach (var step in new[] { 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 30.0 })
                {
                    foreach (var dir in new[] { -1.0, 1.0 })
                    {
                        Array.Copy(stuckAngles, probe.Angles, stuckAngles.Length);
                        probe.Angles[joint] = Math.Max(probe.MinAngles[joint], Math.Min(probe.MaxAngles[joint], probe.Angles[joint] + dir * step));
                        double err = (probe.ForwardKinematics().EePos - home).Length;
                        if (err < best - 1e-6) { best = err; bestDelta = probe.Angles[joint] - stuckAngles[joint]; }
                    }
                }
                Console.WriteLine($"  joint {joint}: best err {best:F3} (delta {bestDelta:F1})");
            }

            // Probe the lift target reachability for each item from the shoulder.
            foreach (var item in items)
            {
                var pickTarget = new Point3D(item.Position.X, Math.Max(item.Position.Y + 0.28, 0.60), item.Position.Z);
                double safeY = Math.Max(0.96 + 1.35, Math.Max(pickTarget.Y, drop.Y) + 0.85);
                var lift = new Point3D(pickTarget.X, safeY, pickTarget.Z);
                var shoulder = new Point3D(0, RobotArmModel.BaseHeight, 0);
                Console.WriteLine($"item {item.PaletteIndex}: pick {Fmt(pickTarget)} distFromShoulder {(pickTarget - shoulder).Length:F3} | lift {Fmt(lift)} distFromShoulder {(lift - shoulder).Length:F3} (max ~5.1)");
            }
        }

        private static AutomationItem NewItem(double x, double y, double z, int palette) => new()
        {
            StartPosition = new Point3D(x, y, z),
            Position = new Point3D(x, y, z),
            PaletteIndex = palette
        };

        private static List<Obstacle> BuildObstacles(Point3D pickup, Point3D drop)
        {
            var list = new List<Obstacle>();
            var random = new Random(7919);
            var forbidden = new[] { pickup, drop };

            for (int i = 0; i < 6; i++)
            {
                Point3D center;
                var halfSize = new Vector3D(0.16, 0.16, 0.16);
                int attempts = 0;

                do
                {
                    double side = i % 2 == 0 ? -1.0 : 1.0;
                    center = new Point3D(
                        -2.6 + random.NextDouble() * 5.2,
                        pickup.Y + 0.5,
                        side * (1.45 + random.NextDouble() * 1.8));
                    attempts++;
                }
                while (attempts < 100 &&
                       (IsNearZone(center, halfSize, forbidden[0], 0.75) ||
                        IsNearZone(center, halfSize, forbidden[1], 0.75) ||
                        Overlaps(list, center, halfSize)));

                list.Add(new Obstacle { Center = center, HalfSize = halfSize });
            }

            return list;
        }

        private static bool IsNearZone(Point3D center, Vector3D halfSize, Point3D zone, double margin)
            => Math.Abs(center.X - zone.X) < halfSize.X + margin &&
               Math.Abs(center.Z - zone.Z) < halfSize.Z + margin;

        private static bool Overlaps(List<Obstacle> obstacles, Point3D center, Vector3D halfSize)
        {
            foreach (var obstacle in obstacles)
            {
                if (Math.Abs(center.X - obstacle.Center.X) < halfSize.X + obstacle.HalfSize.X + 0.15 &&
                    Math.Abs(center.Z - obstacle.Center.Z) < halfSize.Z + obstacle.HalfSize.Z + 0.15)
                    return true;
            }

            return false;
        }

        private static string Fmt(Point3D p) => $"({p.X:F2}, {p.Y:F2}, {p.Z:F2})";
    }
}
