using System;
using System.Linq;
using System.Windows.Media.Media3D;

namespace FkProbe
{
    internal static class Program3
    {
        static readonly double L1 = 2.0, L2 = 1.5, L3 = 1.0, BaseHeight = 0.3, FloorY = 0.30;
        static readonly double[] Min = { -180, -120, -150, -120 };
        static readonly double[] Max = { 180, 120, 150, 120 };

        static (Point3D eePos, Matrix3D eeRot, Point3D[] jo, Vector3D[] ja) FK(double[] angles)
        {
            int n = angles.Length;
            var jo = new Point3D[n]; var ja = new Vector3D[n];
            Matrix3D M = RoboticArm.Kinematics.ForwardKinematics.Translate(0, BaseHeight / 2.0, 0);
            double[] L = { L1, L2, L3, 0.6 };
            Vector3D[] la = { new(0, 1, 0), new(0, 0, 1), new(0, 0, 1), new(0, 0, 1) };
            for (int i = 0; i < n; i++)
            {
                jo[i] = M.Transform(new Point3D(0, 0, 0));
                var a = M.Transform(la[i]); a.Normalize(); ja[i] = a;
                M = RoboticArm.Kinematics.ForwardKinematics.RotateAxisAngle(la[i], angles[i]) * M;
                M = RoboticArm.Kinematics.ForwardKinematics.Translate(L[i], 0, 0) * M;
            }
            var ee = M.Transform(new Point3D(0, 0, 0));
            var rot = M; rot.OffsetX = rot.OffsetY = rot.OffsetZ = 0;
            return (ee, rot, jo, ja);
        }

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

        static double[] Solve3(double[,] A, double[] b)
        {
            double[,] m = new double[3, 4];
            for (int r = 0; r < 3; r++) { for (int c = 0; c < 3; c++) m[r, c] = A[r, c]; m[r, 3] = b[r]; }
            for (int p = 0; p < 3; p++)
            {
                int best = p;
                for (int r = p + 1; r < 3; r++) if (Math.Abs(m[r, p]) > Math.Abs(m[best, p])) best = r;
                if (Math.Abs(m[best, p]) < 1e-12) return null;
                if (best != p) for (int c = p; c < 4; c++) (m[p, c], m[best, c]) = (m[best, c], m[p, c]);
                double d = m[p, p];
                for (int c = p; c < 4; c++) m[p, c] /= d;
                for (int r = 0; r < 3; r++)
                {
                    if (r == p) continue;
                    double f = m[r, p];
                    for (int c = p; c < 4; c++) m[r, c] -= f * m[p, c];
                }
            }
            return new[] { m[0, 3], m[1, 3], m[2, 3] };
        }

        // Clean version: optional greedy yaw (with fk recompute), single delta application.
        public static bool Solve(Point3D target, double[] angles, int maxIter, bool greedyYaw, double yawStepCap)
        {
            int n = angles.Length;
            for (int iter = 0; iter < maxIter; iter++)
            {
                var fk = FK(angles);
                var error = target - fk.eePos;
                if (error.Length < 0.001) return true;

                if (greedyYaw)
                {
                    var ya = (double[])angles.Clone();
                    double des = Math.Atan2(target.Z, target.X) * 180.0 / Math.PI;
                    double yerr = des - angles[0];
                    while (yerr > 180) yerr -= 360;
                    while (yerr < -180) yerr += 360;
                    double step = Math.Max(-yawStepCap, Math.Min(yawStepCap, yerr));
                    ya[0] = Math.Max(Min[0], Math.Min(Max[0], angles[0] + step));
                    if (Safe(ya)) angles[0] = ya[0];
                    fk = FK(angles);
                    error = target - fk.eePos;
                    if (error.Length < 0.001) return true;
                }

                double[,] j = new double[3, n];
                for (int i = 0; i < n; i++)
                {
                    var pa = (double[])angles.Clone(); pa[i] += 0.5;
                    var pp = FK(pa).eePos;
                    j[0, i] = (pp.X - fk.eePos.X) / 0.5;
                    j[1, i] = (pp.Y - fk.eePos.Y) / 0.5;
                    j[2, i] = (pp.Z - fk.eePos.Z) / 0.5;
                }
                var jjt = new double[3, 3];
                for (int r = 0; r < 3; r++)
                    for (int c = 0; c < 3; c++)
                    {
                        double sum = 0;
                        for (int i = 0; i < n; i++) sum += j[r, i] * j[c, i];
                        jjt[r, c] = sum + (r == c ? 0.12 * 0.12 : 0);
                    }
                var x = Solve3(jjt, new[] { error.X, error.Y, error.Z });
                if (x == null) return false;
                var delta = new double[n];
                for (int i = 0; i < n; i++)
                    for (int r = 0; r < 3; r++) delta[i] += j[r, i] * x[r];

                var prev = (double[])angles.Clone();
                bool accepted = false;
                for (int attempt = 0; attempt < 7; attempt++)
                {
                    Array.Copy(prev, angles, n);
                    double gain = 0.8 * Math.Pow(0.5, attempt);
                    for (int i = 0; i < n; i++)
                        angles[i] = Math.Max(Min[i], Math.Min(Max[i], prev[i] + delta[i] * gain));
                    if (Safe(angles) && (target - FK(angles).eePos).Length < error.Length) { accepted = true; break; }
                }
                if (!accepted)
                {
                    Array.Copy(prev, angles, n);
                    double bestE = error.Length; int bj = -1; double bc = 0;
                    foreach (double st in new[] { 0.1, 0.5, 1, 2, 5, 10, 20 })
                        foreach (double dir in new[] { -1.0, 1.0 })
                            for (int joint = 0; joint < n; joint++)
                            {
                                Array.Copy(prev, angles, n);
                                angles[joint] = Math.Max(Min[joint], Math.Min(Max[joint], angles[joint] + dir * st));
                                double e = (target - FK(angles).eePos).Length;
                                if (e < bestE) { bestE = e; bj = joint; bc = dir * st; }
                            }
                    Array.Copy(prev, angles, n);
                    if (bj < 0) return false;
                    angles[bj] = Math.Max(Min[bj], Math.Min(Max[bj], angles[bj] + bc));
                }
            }
            return false;
        }

        static string A(double[] a) => string.Join(", ", a.Select(v => v.ToString("F1").PadLeft(7)));
        static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        public static void Run()
        {
            Console.WriteLine("=== CLEAN SOLVER VARIANTS (fixed FK) ===");
            foreach (var start in new[] { new double[] { 0, -35, 80, -80 }, new double[] { 0, -45, 90, -90 } })
            {
                var fk = FK(start);
                var pickup = new Point3D(fk.eePos.X, 0.30, 0);
                var obj = new Point3D(pickup.X - 0.45, 0.46, pickup.Z - 0.35);
                var path = new[]
                {
                    new Point3D(obj.X, 0.74, obj.Z),
                    new Point3D(obj.X, 1.30, obj.Z),
                    new Point3D(-pickup.X, 1.30, 0),
                    new Point3D(-pickup.X, 0.75, 0)
                };
                foreach (var (label, greedy, cap) in new[] { ("no-yaw", false, 0.0), ("yaw-cap2", true, 2.0), ("yaw-cap8", true, 8.0) })
                {
                    var a = (double[])start.Clone();
                    Console.WriteLine($"-- {label} start {A(start)}");
                    foreach (var wp in path)
                    {
                        int t = 0; double maxd = 0; double pj = a[0];
                        for (; t < 1200; t++)
                        {
                            var cur = FK(a);
                            var d = wp - cur.eePos;
                            var aim = wp;
                            if (d.Length > 0.045) { d.Normalize(); aim = new Point3D(cur.eePos.X + d.X * 0.045, cur.eePos.Y + d.Y * 0.045, cur.eePos.Z + d.Z * 0.045); }
                            Solve(aim, a, 14, greedy, cap);
                            maxd = Math.Max(maxd, Math.Abs(a[0] - pj)); pj = a[0];
                            if ((FK(a).eePos - wp).Length < 0.18) break;
                        }
                        Console.WriteLine($"   wp {P(wp)} ticks {t + 1} dist {(FK(a).eePos - wp).Length:F3} maxdJ1 {maxd:F2} J {A(a)}");
                    }
                }
                Console.WriteLine();
            }
        }
    }
}
