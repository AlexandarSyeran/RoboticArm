using System;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;

namespace FkProbe
{
    internal static class Program
    {
        static readonly double L1 = 2.0, L2 = 1.5, L3 = 1.0;
        static readonly double[] Min = { -180, -120, -150, -120 };
        static readonly double[] Max = { 180, 120, 150, 120 };
        static readonly double BaseHeight = 0.3;
        static readonly double FloorY = 0.30;

        static (Point3D eePos, Matrix3D eeRot, Point3D[] jointOrigins, Vector3D[] jointAxes) FK(double[] angles)
        {
            int n = angles.Length;
            var jointOrigins = new Point3D[n];
            var jointAxes = new Vector3D[n];
            Matrix3D M = ForwardKinematics.Translate(0, BaseHeight / 2.0, 0);
            double[] L = { L1, L2, L3, 0.6 };
            Vector3D[] localAxes = { new(0, 1, 0), new(0, 0, 1), new(0, 0, 1), new(0, 0, 1) };

            for (int i = 0; i < n; i++)
            {
                jointOrigins[i] = M.Transform(new Point3D(0, 0, 0));
                var axisWorld = M.Transform(localAxes[i]);
                axisWorld.Normalize();
                jointAxes[i] = axisWorld;
                var R = ForwardKinematics.RotateAxisAngle(localAxes[i], angles[i]);
                M = R * M;
                var T = ForwardKinematics.Translate(L[i], 0, 0);
                M = T * M;
            }

            var ee = M.Transform(new Point3D(0, 0, 0));
            var rot = M;
            rot.OffsetX = rot.OffsetY = rot.OffsetZ = 0;
            return (ee, rot, jointOrigins, jointAxes);
        }

        // Mirrors MainWindow.IsAutomationCandidateSafe for the no-obstacle case.
        static bool IsSafe(double[] candidateAngles, double[] currentAngles)
        {
            var fk = FK(candidateAngles);
            var current = FK(currentAngles).eePos;
            if (fk.eePos.Y < FloorY) return false;
            for (int i = 0; i < fk.jointOrigins.Length; i++)
            {
                if (i >= 2 && fk.jointOrigins[i].Y < FloorY) return false;
                var end = i + 1 < fk.jointOrigins.Length ? fk.jointOrigins[i + 1] : fk.eePos;
                if (i >= 1 && end.Y < FloorY) return false;
            }
            return true;
        }

        static (bool Success, double Remaining) SolveAutomationPosition(
            double[] angles, Point3D requestedTarget, int iterations, double speed)
        {
            var current = FK(angles);
            var initialError = (current.eePos - requestedTarget).Length;
            var delta = requestedTarget - current.eePos;
            double maxStep = 0.045 * speed / 0.55;
            var target = requestedTarget;
            if (delta.Length > maxStep)
            {
                delta.Normalize();
                target = new Point3D(current.eePos.X + delta.X * maxStep,
                                     current.eePos.Y + delta.Y * maxStep,
                                     current.eePos.Z + delta.Z * maxStep);
            }

            var solved = InverseKinematics6D.SolvePosition(
                target, angles,
                a => { var f = FK(a); return (f.eePos, f.jointOrigins, f.jointAxes); },
                Min, Max, 0.001, iterations, 0.12,
                a => IsSafe(a, angles));

            for (int i = 0; i < angles.Length; i++)
                angles[i] = Math.Max(Min[i], Math.Min(Max[i], angles[i]));

            var actual = FK(angles).eePos;
            double remaining = (actual - requestedTarget).Length;
            bool madeProgress = remaining < initialError - 1e-5;
            bool success = solved || remaining < 0.16 || madeProgress;
            return (success, remaining);
        }

        static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";
        static string A(double[] a) => string.Join(", ", a.Select(v => v.ToString("F1").PadLeft(7)));

        static void Main(string[] args)
        {
            if (args.Length > 0)
            {
                switch (args[0])
                {
                    case "7": Program7.Run(); return;
                    case "9": Program9.Run(); return;
                    case "15": Program15.Run(); return;
                    case "16": Program16.Run(); return;
                    case "17": Program17.Run(); return;
                    case "18": Program18.Run(); return;
                    case "19": Program19.Run(); return;
                    case "20": Program20.Run(); return;
                    case "21": Program21.Run(); return;
                    case "22": Program22.Run(); return;
                    case "23": Program23.Run(); return;
                    case "24": Program24.Run(); return;
                    case "25": Program25.Run(); return;
                    case "26": Program26.Run(); return;
                    case "27": Program27.Run(); return;
                    case "28": Program28.Run(); return;
                    case "29": Program29.Run(); return;
                    case "30": Program30.Run(); return;
                }
            }

            var rot10 = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), 10);
            var rot30 = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), 30);
            var qC = QuatFromMatrix(rot10);
            var qT = QuatFromMatrix(rot30);
            DumpQuat("target * current   (old)", qT * qC);
            DumpQuat("target * conj(current) (fixed)", qT * new Quaternion(-qC.X, -qC.Y, -qC.Z, qC.W));
            Console.WriteLine();
            Program3.Run();
            Console.WriteLine();
            Program4.Run();
            Console.WriteLine();
            Program5.Run();
            Console.WriteLine();
            Program6.Run();
            Console.WriteLine();
            Program7.Run();
            Console.WriteLine();
            Program8.Run();
            Console.WriteLine();
            Program9.Run();
            Console.WriteLine();
            Program10.Run();
            Console.WriteLine();
            Program11.Run();
            Console.WriteLine();
            Program12.Run();
            Console.WriteLine();
            Program13.Run();
            Console.WriteLine();
            Program14.Run();
            Console.WriteLine();
            Program15.Run();
            Console.WriteLine();
            Program16.Run();
            Console.WriteLine();
            Program17.Run();
        }

        static Quaternion QuatFromMatrix(Matrix3D m)
        {
            double trace = m.M11 + m.M22 + m.M33;
            double w, x, y, z;
            if (trace > 0)
            {
                double s = 0.5 / Math.Sqrt(trace + 1.0);
                w = 0.25 / s; x = (m.M32 - m.M23) * s; y = (m.M13 - m.M31) * s; z = (m.M21 - m.M12) * s;
            }
            else if (m.M11 > m.M22 && m.M11 > m.M33)
            {
                double s = 2.0 * Math.Sqrt(1.0 + m.M11 - m.M22 - m.M33);
                w = (m.M32 - m.M23) / s; x = 0.25 * s; y = (m.M12 + m.M21) / s; z = (m.M13 + m.M31) / s;
            }
            else if (m.M22 > m.M33)
            {
                double s = 2.0 * Math.Sqrt(1.0 + m.M22 - m.M11 - m.M33);
                w = (m.M13 - m.M31) / s; x = (m.M12 + m.M21) / s; y = 0.25 * s; z = (m.M23 + m.M32) / s;
            }
            else
            {
                double s = 2.0 * Math.Sqrt(1.0 + m.M33 - m.M11 - m.M22);
                w = (m.M21 - m.M12) / s; x = (m.M13 + m.M31) / s; y = (m.M23 + m.M32) / s; z = 0.25 * s;
            }
            var q = new Quaternion(x, y, z, w); q.Normalize(); return q;
        }

        static void DumpQuat(string label, Quaternion q)
        {
            double ang = 2.0 * Math.Acos(Math.Max(-1, Math.Min(1, q.W))) * 180.0 / Math.PI;
            Console.WriteLine($"{label}: q=({q.X:F3},{q.Y:F3},{q.Z:F3},{q.W:F3}) angle={ang:F1} deg");
        }

        static void MainOld()
        {
            foreach (var start in new[] { new double[] { 0, -35, 80, -80 }, new double[] { 0, -45, 90, -90 } })
            {
                Console.WriteLine("==============================================");
                Console.WriteLine("Initial folded pose: " + A(start));
                var fk = FK(start);
                for (int i = 0; i < fk.jointOrigins.Length; i++)
                    Console.WriteLine($"  joint{i} origin {P(fk.jointOrigins[i])}  axis {fk.jointAxes[i]}");
                Console.WriteLine($"  EE {P(fk.eePos)}");
                Console.WriteLine($"  tool X {fk.eeRot.M11:F3} {fk.eeRot.M12:F3} {fk.eeRot.M13:F3}");

                var pickup = new Point3D(fk.eePos.X, 0.30, 0);
                var drop = new Point3D(-pickup.X, 0.30, 0);
                Console.WriteLine($"  pickup zone {P(pickup)}  drop zone {P(drop)}");

                // Object 1 from ResetAutomationObjects.
                var obj = new Point3D(pickup.X - 0.45, 0.46, pickup.Z - 0.35);
                var pickTarget = new Point3D(obj.X, Math.Max(obj.Y + 0.28, FloorY + 0.30), obj.Z);
                Console.WriteLine("Automation MoveToPick simulation (one tick = 30 ms, speed 55%)");
                Console.WriteLine("  tick |    J1     J2     J3     J4 |  ee pos                    | dist to pick");

                var angles = (double[])start.Clone();
                int tick = 0;
                for (; tick < 400; tick++)
                {
                    var (ok, remaining) = SolveAutomationPosition(angles, pickTarget, 8, 0.55);
                    if (tick % 10 == 0 || remaining < 0.16)
                    {
                        var ee = FK(angles).eePos;
                        double j1 = angles[0];
                        Console.WriteLine($"  {tick,4} | {A(angles)} | {P(ee)} | {remaining:F3} {(remaining < 0.16 ? "<- reached" : "")}");
                    }
                    if (!ok) { Console.WriteLine("  ABORT: no progress"); break; }
                    if (remaining < 0.16) break;
                }
                Console.WriteLine($"  ticks to pick: {tick + 1}, final J1 = {angles[0]:F1}");

                // Now go to drop-zone safe height waypoint to observe base behaviour.
                var lift = new Point3D(obj.X, 1.30, obj.Z);
                var dropHigh = new Point3D(drop.X, 1.30, drop.Z);
                Console.WriteLine("After pick, MoveToDrop waypoints: " + P(lift) + " -> " + P(dropHigh));
                foreach (var wp in new[] { lift, dropHigh })
                {
                    for (int t = 0; t < 600; t++)
                    {
                        var (ok, remaining) = SolveAutomationPosition(angles, wp, 14, 0.55);
                        if (!ok) { Console.WriteLine($"  ABORT at waypoint {P(wp)} tick {t}"); break; }
                        if (remaining < 0.18) break;
                    }
                    Console.WriteLine($"  waypoint {P(wp)} -> J1={angles[0]:F1} dist={((FK(angles).eePos - wp).Length):F3} angles {A(angles)}");
                }
            }
        }
    }
}
