using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Services;

namespace FkProbe
{
    internal static class Program24
    {
        private static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        private static List<AutomationItem> MakeItems(double x0)
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

        private static double Penetration(Point3D a, Vector3D ha, Point3D b, Vector3D hb)
        {
            double ox = Math.Min(a.X + ha.X, b.X + hb.X) - Math.Max(a.X - ha.X, b.X - hb.X);
            double oy = Math.Min(a.Y + ha.Y, b.Y + hb.Y) - Math.Max(a.Y - ha.Y, b.Y - hb.Y);
            double oz = Math.Min(a.Z + ha.Z, b.Z + hb.Z) - Math.Max(a.Z - ha.Z, b.Z - hb.Z);
            return ox > 0 && oy > 0 && oz > 0 ? Math.Min(ox, Math.Min(oy, oz)) : 0;
        }

        public static void Run()
        {
            Console.WriteLine("=== DROP ZONE STACKING + ARM CLEARANCE ===");

            var robot = new RobotArmModel();
            double x0 = 4.192;
            var items = MakeItems(x0);

            var cutting = new CuttingController(robot, items) { SpeedMultiplier = 0.55 };
            cutting.ItemCut += (item, normal) =>
            {
                int index = items.IndexOf(item);
                if (index < 0)
                    return;
                var (keep, offcut) = CutPlanner.Split(item, normal);
                items.RemoveAt(index);
                items.Insert(index, keep);
                items.Insert(index + 1, offcut);
            };
            cutting.Start();
            for (int cutTick = 0; cutTick < 8000 && cutting.IsRunning; cutTick++)
                cutting.Tick();
            Console.WriteLine($"cutting done: items {items.Count}");

            var automation = new AutomationController(
                robot,
                items,
                new List<Obstacle>(),
                new Point3D(x0, 0.30, 0),
                new Point3D(-x0, 0.30, 0),
                new RobotArmModel().ForwardKinematics().EePos)
            {
                SpeedMultiplier = 1.0,
                ArrivalDelay = 0.0,
                ActionDelay = 0.05
            };

            if (!automation.Start())
            {
                Console.WriteLine($"start refused: '{automation.Status}'");
                return;
            }

            const double eeHalf = 0.18;
            double worstEePenetration = 0;
            double worstPayloadPenetration = 0;
            string worstEeAt = string.Empty;
            string worstPayloadAt = string.Empty;
            var placed = new List<AutomationItem>();
            var snapshots = new Dictionary<AutomationItem, Point3D>();

            int tick = 0;
            for (; tick < 120000 && automation.IsRunning; tick++)
            {
                automation.Tick();
                var fk = robot.ForwardKinematics();

                foreach (var item in placed)
                {
                    double eePen = Penetration(fk.EePos, new Vector3D(eeHalf, eeHalf, eeHalf),
                                               item.Position, item.HalfSize);
                    if (eePen > worstEePenetration)
                    {
                        worstEePenetration = eePen;
                        worstEeAt = $"tick {tick} phase {automation.Phase} item {item.PaletteIndex} {P(item.Position)} EE {P(fk.EePos)}";
                    }

                    if (automation.AttachedItem is { } attached && !ReferenceEquals(attached, item))
                    {
                        double plPen = Penetration(attached.Position, attached.HalfSize,
                                                   item.Position, item.HalfSize);
                        if (plPen > worstPayloadPenetration)
                        {
                            worstPayloadPenetration = plPen;
                            worstPayloadAt = $"tick {tick} phase {automation.Phase} payload half {attached.HalfSize.Y:F3} {P(attached.Position)} item {P(item.Position)}";
                        }
                    }
                }

                foreach (var item in items)
                {
                    if (item.Completed && !snapshots.ContainsKey(item))
                        snapshots[item] = item.Position;
                }

                if (placed.Count != snapshots.Count)
                    placed = snapshots.Keys.ToList();
            }

            Console.WriteLine($"automation: completed {items.Count(i => i.Completed)}/{items.Count}, ticks {tick}, status '{automation.Status}'");
            Console.WriteLine($"worst EE penetration: {worstEePenetration:F4} {(worstEePenetration > 0 ? worstEeAt : string.Empty)}");
            Console.WriteLine($"worst payload penetration: {worstPayloadPenetration:F4} {(worstPayloadPenetration > 0 ? worstPayloadAt : string.Empty)}");

            bool moved = items.Any(i => snapshots.TryGetValue(i, out var p) && (i.Position - p).Length > 1e-9);
            Console.WriteLine($"placed objects shifted after release: {moved}");

            Console.WriteLine("final stack:");
            double worstGap = 0;
            string worstGapAt = string.Empty;
            double deepestBelowTable = 0;
            foreach (var item in items.OrderBy(i => i.Position.Y))
            {
                double bottom = item.Position.Y - item.HalfSize.Y;
                double supportTop = 0.30;
                foreach (var other in items)
                {
                    if (ReferenceEquals(other, item))
                        continue;
                    bool overX = Math.Abs(other.Position.X - item.Position.X) <= Math.Abs(item.HalfSize.X + other.HalfSize.X) - 1e-6;
                    bool overZ = Math.Abs(other.Position.Z - item.Position.Z) <= Math.Abs(item.HalfSize.Z + other.HalfSize.Z) - 1e-6;
                    if (!overX || !overZ)
                        continue;
                    double otherTop = other.Position.Y + other.HalfSize.Y;
                    if (otherTop <= bottom + 1e-6)
                        supportTop = Math.Max(supportTop, otherTop);
                }

                double gap = item.Position.Y - item.HalfSize.Y - supportTop;
                Console.WriteLine($"  palette {item.PaletteIndex} side {item.CutSide} pos {P(item.Position)} " +
                                  $"half {item.HalfSize.X:F3},{item.HalfSize.Y:F3},{item.HalfSize.Z:F3} bottom {bottom:F3} support {supportTop:F3} gap {gap:F3}");
                if (gap > worstGap)
                {
                    worstGap = gap;
                    worstGapAt = $"palette {item.PaletteIndex} side {item.CutSide}";
                }

                deepestBelowTable = Math.Max(deepestBelowTable, 0.30 - bottom);
            }

            Console.WriteLine($"largest gap to support: {worstGap:F3} ({worstGapAt})");
            Console.WriteLine($"deepest below table top: {deepestBelowTable:F3}");
            bool ok = worstEePenetration <= 0 && worstPayloadPenetration <= 0 && !moved && deepestBelowTable <= 1e-6;
            Console.WriteLine(ok ? "RESULT: no arm impact, no shifts" : "RESULT: PROBLEMS FOUND");
        }
    }
}
