using System;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;

namespace FkProbe
{
    internal static class Program4
    {
        const double L1 = 2.0, L2 = 1.5, L3 = 1.0, BaseHeight = 0.3, FloorY = 0.30;
        static readonly double[] Min = { -180, -120, -150, -120 };
        static readonly double[] Max = { 180, 120, 150, 120 };

        public static (Point3D eePos, Matrix3D eeRot, Point3D[] jo, Vector3D[] ja) FK2(double[] angles) => FK(angles);
        public static Quaternion Quat2(Matrix3D m) => Quat(m);

        static (Point3D eePos, Matrix3D eeRot, Point3D[] jo, Vector3D[] ja) FK(double[] angles)
        {
            int n = angles.Length;
            var jo = new Point3D[n]; var ja = new Vector3D[n];
            Matrix3D M = ForwardKinematics.Translate(0, BaseHeight / 2.0, 0);
            double[] L = { L1, L2, L3, 0.6 };
            Vector3D[] la = { new(0, 1, 0), new(0, 0, 1), new(0, 0, 1), new(0, 0, 1) };
            for (int i = 0; i < n; i++)
            {
                jo[i] = M.Transform(new Point3D(0, 0, 0));
                var a = M.Transform(la[i]); a.Normalize(); ja[i] = a;
                M = ForwardKinematics.RotateAxisAngle(la[i], angles[i]) * M;
                M = ForwardKinematics.Translate(L[i], 0, 0) * M;
            }
            var ee = M.Transform(new Point3D(0, 0, 0));
            var rot = M; rot.OffsetX = rot.OffsetY = rot.OffsetZ = 0;
            return (ee, rot, jo, ja);
        }

        // Mirrors MainWindow.IsAutomationCandidateSafe with no obstacles present.
        static bool Safe(double[] cand)
        {
            var fk = FK(cand);
            if (fk.eePos.Y < FloorY) return false;
            for (int i = 0; i < fk.jo.Length; i++)
            {
                if (i >= 2 && fk.jo[i].Y < FloorY) return false;
                var end = i + 1 < fk.jo.Length ? fk.jo[i + 1] : fk.eePos;
                if (i >= 1 && end.Y < FloorY) return false;
            }
            return true;
        }

        static double SolveTick(double[] angles, Point3D requested, int iterations, double speed)
        {
            var current = FK(angles);
            var delta = requested - current.eePos;
            double maxStep = 0.045 * speed / 0.55;
            var target = requested;
            if (delta.Length > maxStep)
            {
                delta.Normalize();
                target = new Point3D(current.eePos.X + delta.X * maxStep,
                                     current.eePos.Y + delta.Y * maxStep,
                                     current.eePos.Z + delta.Z * maxStep);
            }
            InverseKinematics6D.SolvePosition(target, angles,
                a => { var f = FK(a); return (f.eePos, f.jo, f.ja); },
                Min, Max, 0.001, iterations, 0.12, Safe);
            for (int i = 0; i < angles.Length; i++)
                angles[i] = Math.Max(Min[i], Math.Min(Max[i], angles[i]));
            return (FK(angles).eePos - requested).Length;
        }

        static string A(double[] a) => string.Join(", ", a.Select(v => v.ToString("F1").PadLeft(7)));
        static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        static void MoveTo(string label, double[] angles, Point3D wp, int iterations, double speed, int maxTicks)
        {
            double prev = angles[0], maxd = 0;
            int t = 0;
            for (; t < maxTicks; t++)
            {
                double rem = SolveTick(angles, wp, iterations, speed);
                maxd = Math.Max(maxd, Math.Abs(angles[0] - prev)); prev = angles[0];
                if (rem < 0.18) break;
            }
            Console.WriteLine($"  {label,-12} ticks {t + 1,4}  dist {(FK(angles).eePos - wp).Length:F3}  maxdJ1/tick {maxd:F2}  J {A(angles)}");
        }

        public static void Run()
        {
            Console.WriteLine("=== PATCHED PRODUCTION SOLVER, FULL CYCLE (55% speed, 30 ms/tick) ===");
            var homeAngles = new double[] { 0, -45, 90, -90 };
            var fkHome = FK(homeAngles);
            var pickup = new Point3D(fkHome.eePos.X, 0.30, 0);
            var drop = new Point3D(-pickup.X, 0.30, 0);
            var obj = new Point3D(pickup.X - 0.45, 0.46, pickup.Z - 0.35);

            var a = (double[])homeAngles.Clone();
            MoveTo("to object", a, new Point3D(obj.X, 0.74, obj.Z), 8, 0.55, 600);
            MoveTo("lift", a, new Point3D(obj.X, 1.30, obj.Z), 14, 0.55, 300);
            MoveTo("drop high", a, new Point3D(drop.X, 1.30, drop.Z), 14, 0.55, 1200);
            MoveTo("drop low", a, new Point3D(drop.X, 0.75, drop.Z), 14, 0.55, 300);
            MoveTo("home", a, fkHome.eePos, 8, 0.55, 1200);
            Console.WriteLine($"  final home distance {(FK(a).eePos - fkHome.eePos).Length:F3}");

            Console.WriteLine();
            Console.WriteLine("=== PATCHED 6D IK: REACHABLE TARGET POSE (position + orientation) ===");
            var start = (double[])homeAngles.Clone();
            var startFk = FK(start);
            var goalAngles = new double[] { 5, -60, 105, -95 };
            var goalFk = FK(goalAngles);
            var targetRot = goalFk.eeRot;
            bool ok = InverseKinematics6D.Solve6D(goalFk.eePos, targetRot, start,
                ang => { var f = FK(ang); return (f.eePos, f.eeRot, f.jo, f.ja); },
                Min, Max, true, 1e-3, 1e-2, 400, 0.08);
            var endFk = FK(start);
            var qC = Quat(endFk.eeRot);
            var qT = Quat(targetRot);
            var qE = new Quaternion(-qC.X, -qC.Y, -qC.Z, qC.W) * qT;
            if (qE.W < 0) qE = new Quaternion(-qE.X, -qE.Y, -qE.Z, -qE.W);
            double errDeg = 2 * Math.Acos(Math.Min(1, Math.Abs(qE.W))) * 180 / Math.PI;
            Console.WriteLine($"  solved={ok} final orientation error={errDeg:F2} deg  pos error={(endFk.eePos - goalFk.eePos).Length:F4}  J {A(start)}");

            Console.WriteLine();
            Console.WriteLine("=== OLD vs FIXED ORIENTATION ERROR FORMULA ===");
            var rot10 = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), 10);
            var rot30 = ForwardKinematics.RotateAxisAngle(new Vector3D(0, 0, 1), 30);
            var a10 = Quat(rot10); var a30 = Quat(rot30);
            Console.WriteLine($"  true difference = 20 deg");
            Dump("target * current        (original)", a30 * a10);
            Dump("conj(current) * target  (fixed)", new Quaternion(-a10.X, -a10.Y, -a10.Z, a10.W) * a30);
            Dump("target * conj(current)  (wrong side)", a30 * new Quaternion(-a10.X, -a10.Y, -a10.Z, a10.W));
        }

        static void Dump(string label, Quaternion q)
        {
            if (q.W < 0) q = new Quaternion(-q.X, -q.Y, -q.Z, -q.W);
            double deg = 2 * Math.Acos(Math.Min(1, Math.Abs(q.W))) * 180 / Math.PI;
            Console.WriteLine($"  {label}: {deg,6:F2} deg  q=({q.X:F3},{q.Y:F3},{q.Z:F3},{q.W:F3})");
        }

        static Quaternion Quat(Matrix3D m)
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
    }
}
