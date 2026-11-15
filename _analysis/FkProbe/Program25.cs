using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program25
    {
        // Pose read from the joint sliders in the screenshot.
        static readonly double[] Pose = { 5.4, 18.1, -40.6, 90.9, 40.0 };

        static List<AutomationItem> MakeItems(double x0)
        {
            var positions = new[]
            {
                new Point3D(x0 - 0.45, 0.46, -0.35),
                new Point3D(x0 + 0.45, 0.46, -0.35),
                new Point3D(x0 - 0.45, 0.46, 0.35),
                new Point3D(x0 + 0.45, 0.46, 0.35)
            };

            var items = new List<AutomationItem>();
            for (int i = 0; i < positions.Length; i++)
            {
                items.Add(new AutomationItem
                {
                    StartPosition = positions[i],
                    Position = positions[i],
                    PaletteIndex = i
                });
            }

            return items;
        }

        static bool TickUntilIdle(AutomationController automation, int limit, out int ticks)
        {
            ticks = 0;
            while (ticks < limit && automation.IsRunning)
            {
                automation.Tick();
                ticks++;
            }

            return !automation.IsRunning;
        }

        public static void Run()
        {
            Console.WriteLine("=== Proposed initial pose { 5.4, 18.1, -40.6, 90.9, 40 } ===");

            var robot = new RobotArmModel();
            Array.Copy(Pose, robot.Angles, Pose.Length);
            var fk = robot.ForwardKinematics();
            Console.WriteLine($"  EE {P(fk.EePos)}");
            for (int i = 0; i < fk.JointOrigins.Length; i++)
                Console.WriteLine($"  joint{i} origin {P(fk.JointOrigins[i])}");

            var folded = new RobotArmModel();
            Console.WriteLine($"  folded EE for reference {P(folded.ForwardKinematics().EePos)}");

            double x0 = fk.EePos.X;
            var pickup = new Point3D(x0, 0.30, 0);
            var drop = new Point3D(-x0, 0.30, 0);
            var items = MakeItems(x0);

            var automation = new AutomationController(robot, items, new List<Obstacle>(), pickup, drop, fk.EePos)
            {
                SpeedMultiplier = 1.0,
                ArrivalDelay = 0.0,
                ActionDelay = 0.05
            };

            bool finished = false;
            automation.Finished += () => finished = true;

            Console.WriteLine($"  Start operation accepted: {automation.Start()} '{automation.Status}'");
            TickUntilIdle(automation, 120000, out int ticks);
            int completed = items.Count(i => i.Completed);
            Console.WriteLine($"  automation: {completed}/4, ticks {ticks} (~{ticks * 0.03:F0}s), " +
                              $"finished={finished}, status '{automation.Status}', diag '{automation.Diagnostic}'");

            // Manual flow from the same start pose.
            robot.ResetPose();
            Array.Copy(Pose, robot.Angles, Pose.Length);
            items.Clear();
            items.AddRange(MakeItems(x0));
            automation.ResetState();

            bool pick1 = automation.StartManualPick();
            TickUntilIdle(automation, 20000, out int pickTicks);
            bool held = automation.AttachedItem != null;
            Console.WriteLine($"  manual pick 1: ok={pick1} held={held} ticks {pickTicks} status '{automation.Status}'");

            bool place = automation.StartManualPlace();
            TickUntilIdle(automation, 30000, out int placeTicks);
            Console.WriteLine($"  manual place: ok={place} ticks {placeTicks} EE {P(robot.ForwardKinematics().EePos)} " +
                              $"status '{automation.Status}' diag '{automation.Diagnostic}'");

            bool pick2 = automation.StartManualPick();
            TickUntilIdle(automation, 30000, out int pick2Ticks);
            Console.WriteLine($"  manual pick 2: ok={pick2} held={automation.AttachedItem != null} ticks {pick2Ticks} " +
                              $"status '{automation.Status}' diag '{automation.Diagnostic}'");

            bool ok = completed == 4 && finished && pick1 && held && place && pick2 &&
                      automation.AttachedItem != null;
            Console.WriteLine(ok ? "RESULT: pose works for start operation and manual flow" : "RESULT: PROBLEM");

            // Variant: keep the existing pickup/drop zones (x = 4.192) so the
            // scene does not shift, and only start/return from the new pose.
            robot.ResetPose();
            Array.Copy(Pose, robot.Angles, Pose.Length);
            items.Clear();
            items.AddRange(MakeItems(4.192));
            automation.ResetState();
            automation.Stop("reset");
            robot.ResetPose();
            Array.Copy(Pose, robot.Angles, Pose.Length);

            var pinnedPickup = new Point3D(4.192, 0.30, 0);
            var pinnedDrop = new Point3D(-4.192, 0.30, 0);
            var pinned = new AutomationController(
                robot, items, new List<Obstacle>(), pinnedPickup, pinnedDrop, fk.EePos)
            {
                SpeedMultiplier = 1.0,
                ArrivalDelay = 0.0,
                ActionDelay = 0.05
            };

            bool pinnedFinished = false;
            pinned.Finished += () => pinnedFinished = true;
            bool pinnedStarted = pinned.Start();
            TickUntilIdle(pinned, 120000, out int pinnedTicks);
            int pinnedCompleted = items.Count(i => i.Completed);
            Console.WriteLine($"  pinned zones: start={pinnedStarted}, {pinnedCompleted}/4, " +
                              $"ticks {pinnedTicks} (~{pinnedTicks * 0.03:F0}s), finished={pinnedFinished}, " +
                              $"status '{pinned.Status}'");

            bool okPinned = pinnedStarted && pinnedCompleted == 4 && pinnedFinished;
            Console.WriteLine(okPinned ? "RESULT: pinned scene works with the new pose" : "RESULT: pinned scene PROBLEM");

            // Manual flow with the pinned scene.
            robot.ResetPose();
            items.Clear();
            items.AddRange(MakeItems(4.192));
            pinned.ResetState();

            bool pPick1 = pinned.StartManualPick();
            TickUntilIdle(pinned, 20000, out int pPickTicks);
            bool pPick1Done = pinned.AttachedItem != null;
            bool pPlace = pinned.StartManualPlace();
            TickUntilIdle(pinned, 30000, out int pPlaceTicks);
            bool pPick2 = pinned.StartManualPick();
            TickUntilIdle(pinned, 30000, out int pPick2Ticks);
            Console.WriteLine($"  pinned manual: pick1={pPick1}/{pPick1Done} ({pPickTicks}), " +
                              $"place={pPlace} ({pPlaceTicks}) '{pinned.Status}', " +
                              $"pick2={pPick2}/{pinned.AttachedItem != null} ({pPick2Ticks})");

            bool okPinnedManual = pPick1 && pPick1Done && pPlace && pPick2 && pinned.AttachedItem != null;
            Console.WriteLine(okPinnedManual ? "RESULT: pinned manual flow OK" : "RESULT: pinned manual PROBLEM");
        }

        static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";
    }
}
