using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program21
    {
        private static string Fmt(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        private static AutomationItem Item(double x, double y, double z, int palette)
        {
            var position = new Point3D(x, y, z);
            return new AutomationItem { StartPosition = position, Position = position, PaletteIndex = palette };
        }

        public static void Run()
        {
            Console.WriteLine("=== MANUAL PICK / PLACE, NO OBSTACLES ===");

            var robot = new RobotArmModel();
            var folded = robot.ForwardKinematics();
            var pickup = new Point3D(4.192, 0.30, 0);
            var drop = new Point3D(-pickup.X, pickup.Y, pickup.Z);

            var items = new List<AutomationItem>
            {
                Item(pickup.X - 0.45, 0.46, pickup.Z - 0.35, 0),
                Item(pickup.X + 0.45, 0.46, pickup.Z - 0.35, 1),
                Item(pickup.X - 0.45, 0.46, pickup.Z + 0.35, 2),
                Item(pickup.X + 0.45, 0.46, pickup.Z + 0.35, 3)
            };

            var obstacles = new List<Obstacle>();
            var controller = new AutomationController(robot, items, obstacles, pickup, drop, folded.EePos)
            {
                ArrivalDelay = 2.0,
                ActionDelay = 0.7,
                SpeedMultiplier = 0.55,
                TickSeconds = 0.03
            };

            Console.WriteLine($"home EE {Fmt(folded.EePos)}");
            bool ok = controller.StartManualPick();
            Console.WriteLine($"pick1 -> {ok} '{controller.Status}'");
            RunToIdle(controller, robot, 6000);
            Console.WriteLine($"pick1 done: held={controller.AttachedItem != null} EE {Fmt(robot.ForwardKinematics().EePos)}");

            bool placed = controller.StartManualPlace();
            Console.WriteLine($"place -> {placed} '{controller.Status}'");
            RunToIdle(controller, robot, 6000);
            Console.WriteLine($"place done: EE {Fmt(robot.ForwardKinematics().EePos)} angles [{string.Join(", ", Array.ConvertAll(robot.Angles, a => a.ToString("F1")))}]");

            bool pick2 = controller.StartManualPick();
            Console.WriteLine($"pick2 -> {pick2} '{controller.Status}' diag='{controller.Diagnostic}'");
            RunToIdle(controller, robot, 6000);
            Console.WriteLine($"pick2 done: held={controller.AttachedItem != null} EE {Fmt(robot.ForwardKinematics().EePos)}");

            if (!pick2)
            {
                var current = robot.ForwardKinematics().EePos;
                var plan = TransportPathPlanner.Plan(current,
                    new Point3D(pickup.X - 0.45, 0.74, pickup.Z - 0.35),
                    obstacles, new Vector3D(), 0.20, 1.0, Math.Max(2.6, current.Y + 0.85), 4.74);
                Console.WriteLine($"direct Plan from {Fmt(current)} -> {plan.Mode}");
            }
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
