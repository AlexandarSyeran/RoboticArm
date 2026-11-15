using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program7
    {
        private static string Fmt(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
        }

        public static void Run()
        {
            Console.WriteLine("=== MANUAL PICK / PLACE ===");

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

            var obstacles = new List<Obstacle>
            {
                new() { Center = new Point3D(1.5, 0.80, 2.2), HalfSize = new Vector3D(0.16, 0.16, 0.16) },
                new() { Center = new Point3D(-1.5, 0.80, -2.2), HalfSize = new Vector3D(0.16, 0.16, 0.16) }
            };

            var controller = new AutomationController(robot, items, obstacles, pickup, drop, folded.EePos)
            {
                ArrivalDelay = 2.0,
                ActionDelay = 0.7,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            Array.Copy(robot.InitialFoldedAngles, robot.Angles, robot.Angles.Length);
            robot.Angles[0] = 70;
            robot.Angles[1] = 30;
            robot.Angles[3] = 20;
            Console.WriteLine($"start pose angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F0")))}] EE {Fmt(robot.ForwardKinematics().EePos)}");

            bool ok = controller.StartManualPick();
            Console.WriteLine($"StartManualPick -> {ok} '{controller.Status}'");
            RunToIdle(controller, robot, 6000);

            var held = controller.AttachedItem;
            Console.WriteLine($"pick done: held={(held != null)} EE {Fmt(robot.ForwardKinematics().EePos)} status='{controller.Status}'");
            if (held == null)
                return;
            Console.WriteLine($"  held item sector {held.PaletteIndex} pos {Fmt(held.Position)}");

            bool started = controller.StartManualPlace();
            Console.WriteLine($"StartManualPlace -> {started} '{controller.Status}'");
            RunToIdle(controller, robot, 6000);

            double dx = held.Position.X - drop.X;
            double dz = held.Position.Z - drop.Z;
            Console.WriteLine($"place done: item {Fmt(held.Position)} drop {Fmt(drop)} dxz={Math.Sqrt(dx * dx + dz * dz):F3} completed={held.Completed} status='{controller.Status}'");
            Console.WriteLine($"  final angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F1")))}]");

            // Pick the next available object without moving the arm again.
            bool pick2 = controller.StartManualPick();
            Console.WriteLine($"second StartManualPick -> {pick2} '{controller.Status}'");
            RunToIdle(controller, robot, 6000);
            Console.WriteLine($"second pick done: held={(controller.AttachedItem != null)} status='{controller.Status}'");
        }

        private static void RunToIdle(AutomationController controller, RobotArmModel robot, int maxTicks)
        {
            string last = controller.Status;
            int stall = 0;
            for (int tick = 0; tick < maxTicks && controller.IsRunning; tick++)
            {
                controller.Tick();
                if (controller.Status != last)
                {
                    last = controller.Status;
                    stall = 0;
                    Console.WriteLine($"  t={tick * 0.03,6:F2}s {controller.Phase,-20} {controller.Status}");
                }
                else if (++stall > 2500)
                {
                    Console.WriteLine($"  STALL at {controller.Phase} '{controller.Status}' EE {Fmt(robot.ForwardKinematics().EePos)}");
                    return;
                }
            }
        }
    }
}
