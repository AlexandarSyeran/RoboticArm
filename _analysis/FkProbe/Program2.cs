using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;

namespace FkProbe
{
    internal static class Program2
    {
        const double L1 = 2.0, L2 = 1.5, L3 = 1.0, BaseHeight = 0.3, FloorY = 0.30;
        static readonly double[] Min = { -180, -120, -150, -120 };
        static readonly double[] Max = { 180, 120, 150, 120 };

        static (Point3D eePos, Matrix3D eeRot, Point3D[] jointOrigins, Vector3D[] jointAxes) FK(double[] angles)
        {
            int n = angles.Length;
            var jo = new Point3D[n];
            var ja = new Vector3D[n];
            Matrix3D M = ForwardKinematics.Translate(0, BaseHeight / 2.0, 0);
            double[] L = { L1, L2, L3, 0.6 };
            Vector3D[] localAxes = { new(0, 1, 0), new(0, 0, 1), new(0, 0, 1), new(0, 0, 1) };
            for (int i = 0; i < n; i++)
            {
                jo[i] = M.Transform(new Point3D(0, 0, 0));
                var a = M.Transform(localAxes[i]); a.Normalize(); ja[i] = a;
                M = ForwardKinematics.RotateAxisAngle(localAxes[i], angles[i]) * M;
                M = ForwardKinematics.Translate(L[i], 0, 0) * M;
            }
            var ee = M.Transform(new Point3D(0, 0, 0));
            var rot = M; rot.OffsetX = rot.OffsetY = rot.OffsetZ = 0;
            return (ee, rot, jo, ja);
        }

        static bool Safe(double[] cand, double[] cur)
        {
            var fk = FK(cand);
            if (fk.eePos.Y < FloorY) return false;
            for (int i = 0; i < fk.jointOrigins.Length; i++)
            {
                if (i >= 2 && fk.jointOrigins[i].Y < FloorY) return false;
                var end = i + 1 < fk.jointOrigins.Length ? fk.jointOrigins[i + 1] : fk.eePos;
                if (i >= 1 && end.Y < FloorY) return false;
            }
            return true;
        }

        static double NormDeg(double a)
        {
            while (a > 180) a -= 360;
            while (a < -180) a += 360;
            return a;
        }

        // Current production solver (zip), kept for comparison.
        static bool SolvePositionCurrent(Point3D targetPos, double[] anglesDeg, int maxIter, double yawGreedy)
        {
            int n = anglesDeg.Length;
            for (int iter = 0; iter < maxIter; iter++)
            {
                var fk = FK(anglesDeg);
                var error = new Vector3D(targetPos.X - fk.eePos.X, targetPos.Y - fk.eePos.Y, targetPos.Z - fk.eePos.Z);
                if (error.Length < 0.001) return true;

                if (yawGreedy > 0)
                {
                    var ya = (double[])anglesDeg.Clone();
                    double bestErr = error.Length, bestYaw = anglesDeg[0];
                    foreach (var step in new[] { -8.0, -4.0, -2.0, -1.0, 1.0, 2.0, 4.0, 8.0 })
                    {
                        ya[0] = Math.Max(Min[0], Math.Min(Max[0], anglesDeg[0] + step));
                        double e = (targetPos - FK(ya).eePos).Length;
                        if (Safe(ya, anglesDeg) && e < bestErr) { bestErr = e; bestYaw = ya[0]; }
                    }
                    anglesDeg[0] = bestYaw;
                }

                double[,] j = new double[3, n];
                for (int i = 0; i < n; i++)
                {
                    var pa = (double[])anglesDeg.Clone(); pa[i] += 0.5;
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

                var prev = (double[])anglesDeg.Clone();
                bool accepted = false;
                for (int attempt = 0; attempt < 7; attempt++)
                {
                    Array.Copy(prev, anglesDeg, n);
                    double gain = 0.8 * Math.Pow(0.5, attempt);
                    for (int i = 0; i < n; i++) anglesDeg[i] += delta[i] * gain;
                    // NOTE: original also adds the step a second time here (clamped to 5 deg).
                    for (int i = 0; i < n; i++)
                    {
                        double step = Math.Max(-5.0, Math.Min(5.0, delta[i] * gain));
                        anglesDeg[i] += step;
                    }
                    var cand = FK(anglesDeg).eePos;
                    if ((targetPos - cand).Length < error.Length) { accepted = true; break; }
                }
                if (!accepted)
                {
                    Array.Copy(prev, anglesDeg, n);
                    double bestE = error.Length; int bj = -1; double bc = 0;
                    foreach (double step in new[] { 0.1, 0.5, 1, 2, 5, 10, 20 })
                        foreach (double dir in new[] { -1.0, 1.0 })
                            for (int joint = 0; joint < n; joint++)
                            {
                                Array.Copy(prev, anglesDeg, n);
                                anglesDeg[joint] += dir * step;
                                for (int i = 0; i < n; i++) anglesDeg[i] = Math.Max(Min[i], Math.Min(Max[i], anglesDeg[i]));
                                var cand = FK(anglesDeg).eePos;
                                double e = (targetPos - cand).Length;
                                if (e < bestE) { bestE = e; bj = joint; bc = dir * step; }
                            }
                    Array.Copy(prev, anglesDeg, n);
                    if (bj < 0) return false;
                    anglesDeg[bj] += bc;
                    for (int i = 0; i < n; i++) anglesDeg[i] = Math.Max(Min[i], Math.Min(Max[i], anglesDeg[i]));
                }
            }
            return false;
        }

        // Proposed: rate-limited base alignment + base-locked 3-DOF numeric IK.
        static (bool converged, bool aligning) SolvePositionYawFirst(
            Point3D targetPos, double[] anglesDeg, int maxIter,
            double maxYawStepDeg, out double yawErrorAfter)
        {
            double desiredYaw = Math.Atan2(targetPos.Z, targetPos.X) * 180.0 / Math.PI;
            double yawErr = NormDeg(desiredYaw - anglesDeg[0]);
            yawErrorAfter = yawErr;
            if (Math.Abs(yawErr) > 1.0)
            {
                anglesDeg[0] += Math.Max(-maxYawStepDeg, Math.Min(maxYawStepDeg, yawErr));
                return (false, true);
            }
            anglesDeg[0] = desiredYaw; // snap the last degree

            int n = anglesDeg.Length;
            var lockedMin = (double[])Min.Clone();
            var lockedMax = (double[])Max.Clone();
            lockedMin[0] = lockedMax[0] = anglesDeg[0];

            for (int iter = 0; iter < maxIter; iter++)
            {
                var fk = FK(anglesDeg);
                var error = targetPos - fk.eePos;
                if (error.Length < 0.001) return (true, false);

                double[,] j = new double[3, n];
                for (int i = 1; i < n; i++)
                {
                    var pa = (double[])anglesDeg.Clone(); pa[i] += 0.5;
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
                        for (int i = 1; i < n; i++) sum += j[r, i] * j[c, i];
                        jjt[r, c] = sum + (r == c ? 0.12 * 0.12 : 0);
                    }
                var x = Solve3(jjt, new[] { error.X, error.Y, error.Z });
                if (x == null) return (false, false);
                var delta = new double[n];
                for (int i = 1; i < n; i++)
                    for (int r = 0; r < 3; r++) delta[i] += j[r, i] * x[r];

                var prev = (double[])anglesDeg.Clone();
                bool accepted = false;
                for (int attempt = 0; attempt < 7; attempt++)
                {
                    Array.Copy(prev, anglesDeg, n);
                    double gain = 0.8 * Math.Pow(0.5, attempt);
                    for (int i = 1; i < n; i++)
                        anglesDeg[i] = Math.Max(lockedMin[i], Math.Min(lockedMax[i], prev[i] + delta[i] * gain));
                    var cand = FK(anglesDeg).eePos;
                    if (Safe(anglesDeg, prev) && (targetPos - cand).Length < error.Length) { accepted = true; break; }
                }
                if (!accepted) return (false, false);
            }
            return (false, false);
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

        static string A(double[] a) => string.Join(", ", a.Select(v => v.ToString("F1").PadLeft(7)));
        static string P(Point3D p) => $"({p.X,7:F3},{p.Y,7:F3},{p.Z,7:F3})";

        static double[] Simulate(string label, Point3D[] targets, double[] start, int maxTicks, bool legacy)
        {
            var angles = (double[])start.Clone();
            Console.WriteLine($"--- {label} ---");
            foreach (var wp in targets)
            {
                double prevJ1 = angles[0];
                double maxDJ1 = 0;
                int t = 0;
                for (; t < maxTicks; t++)
                {
                    bool ok;
                    if (legacy)
                    {
                        var cur = FK(angles);
                        var d = wp - cur.eePos;
                        var aim = wp;
                        if (d.Length > 0.045) { d.Normalize(); aim = new Point3D(cur.eePos.X + d.X * 0.045, cur.eePos.Y + d.Y * 0.045, cur.eePos.Z + d.Z * 0.045); }
                        var solved = SolvePositionCurrent(aim, angles, 14, 8.0);
                        for (int i = 0; i < 4; i++) angles[i] = Math.Max(Min[i], Math.Min(Max[i], angles[i]));
                        double rem = (FK(angles).eePos - wp).Length;
                        double init = (cur.eePos - wp).Length;
                        ok = solved || rem < 0.16 || rem < init - 1e-5;
                        maxDJ1 = Math.Max(maxDJ1, Math.Abs(angles[0] - prevJ1));
                        prevJ1 = angles[0];
                        if (rem < 0.18) break;
                        if (!ok) break;
                    }
                    else
                    {
                        var cur = FK(angles);
                        var d = wp - cur.eePos;
                        var aim = wp;
                        if (d.Length > 0.045) { d.Normalize(); aim = new Point3D(cur.eePos.X + d.X * 0.045, cur.eePos.Y + d.Y * 0.045, cur.eePos.Z + d.Z * 0.045); }
                        var (_, aligning) = SolvePositionYawFirst(aim, angles, 14, 2.5, out _);
                        // keep yaw step limit vs target direction, not per-call sliver
                        double rem = (FK(angles).eePos - wp).Length;
                        maxDJ1 = Math.Max(maxDJ1, Math.Abs(angles[0] - prevJ1));
                        prevJ1 = angles[0];
                        if (!aligning && rem < 0.18) break;
                        ok = true;
                    }
                    if (!ok) break;
                }
                var final = FK(angles).eePos;
                Console.WriteLine($"wp {P(wp)} -> ticks {t + 1}, dist {((final - wp).Length):F3}, max dJ1/tick {maxDJ1:F2}, angles {A(angles)}, ee {P(final)}");
            }
            return angles;
        }

        public static void Run()
        {
            // Can base-locked IK converge if it starts on the correct (mirrored) side?
            {
                Console.WriteLine("MIRROR-SEED TEST");
                var dropHigh = new Point3D(-4.192, 1.30, 0.0);
                foreach (var seed in new[]
                {
                    new double[] { 180, -40, 80, -90 },
                    new double[] { 180, 55, -110, 100 },
                    new double[] { 180, 45, -90, 90 },
                    new double[] { 180, 30, -60, 60 }
                })
                {
                    var a = (double[])seed.Clone();
                    bool aligned = false;
                    for (int t = 0; t < 200; t++)
                    {
                        var (conv, aligning) = SolvePositionYawFirst(dropHigh, a, 14, 2.5, out _);
                        aligned |= !aligning;
                        if (conv) break;
                    }
                    Console.WriteLine($"seed {A(seed)} -> angles {A(a)}, ee {P(FK(a).eePos)}, dist {(FK(a).eePos - dropHigh).Length:F3}");
                }
                Console.WriteLine();
            }

            foreach (var start in new[] { new double[] { 0, -35, 80, -80 }, new double[] { 0, -45, 90, -90 } })
            {
                Console.WriteLine("==========================================================");
                Console.WriteLine("start " + A(start));
                var fk = FK(start);
                var pickup = new Point3D(fk.eePos.X, 0.30, 0);
                var drop = new Point3D(-pickup.X, 0.30, 0);
                var obj = new Point3D(pickup.X - 0.45, 0.46, pickup.Z - 0.35);
                var pickAim = new Point3D(obj.X, 0.74, obj.Z);
                var lift = new Point3D(obj.X, 1.30, obj.Z);
                var dropHigh = new Point3D(drop.X, 1.30, drop.Z);
                var dropLow = new Point3D(drop.X, 0.75, drop.Z);

                var legacyAngles = Simulate("LEGACY solver", new[] { pickAim, lift, dropHigh, dropLow }, start, 800, true);
                Console.WriteLine();
                var fixedAngles = Simulate("YAW-FIRST solver", new[] { pickAim, lift, dropHigh, dropLow }, start, 800, false);
                Console.WriteLine();
            }
        }
    }
}
