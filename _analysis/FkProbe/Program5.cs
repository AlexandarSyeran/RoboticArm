using System;
using System.Windows.Media.Media3D;
using RoboticArm.Kinematics;

namespace FkProbe
{
    internal static class Program5
    {
        public static void Run()
        {
            var angles = new double[] { 0, -45, 90, -90 };
            var fk = Program4.FK2(angles);
            Console.WriteLine("=== ANALYTIC vs NUMERIC JACOBIAN at home pose ===");
            double eps = 0.5 * Math.PI / 180.0;
            for (int i = 0; i < angles.Length; i++)
            {
                var p = (double[])angles.Clone();
                p[i] += eps;
                var fk2 = Program4.FK2(p);

                var numericLin = (fk2.eePos - fk.eePos) / eps;
                var analyticLin = Vector3D.CrossProduct(fk.ja[i], fk.eePos - fk.jo[i]);

                var q0 = Program4.Quat2(fk.eeRot);
                var q1 = Program4.Quat2(fk2.eeRot);
                var qd = q1 * new Quaternion(-q0.X, -q0.Y, -q0.Z, q0.W);
                if (qd.W < 0) qd = new Quaternion(-qd.X, -qd.Y, -qd.Z, -qd.W);
                var v = new Vector3D(qd.X, qd.Y, qd.Z);
                double s = Math.Sqrt(Math.Max(0, 1 - qd.W * qd.W));
                var numericAng = s < 1e-9 ? new Vector3D() : v / s * (2 * Math.Acos(Math.Min(1, qd.W)) / eps);

                Console.WriteLine($" joint{i} axis {fk.ja[i]}");
                Console.WriteLine($"   lin numeric {numericLin}  analytic {analyticLin}");
                Console.WriteLine($"   ang numeric {numericAng}  analytic {fk.ja[i]}");
            }

            // What does the solver's error vector look like at the start?
            var goal = new double[] { 5, -60, 105, -95 };
            var g = Program4.FK2(goal);
            var tgt = new Quaternion();
            var qc = Program4.Quat2(fk.eeRot);
            var qt = Program4.Quat2(g.eeRot);
            var qe = qt * new Quaternion(-qc.X, -qc.Y, -qc.Z, qc.W);
            if (qe.W < 0) qe = new Quaternion(-qe.X, -qe.Y, -qe.Z, -qe.W);
            double ang = 2 * Math.Acos(Math.Min(1, qe.W));
            var ax = new Vector3D(qe.X, qe.Y, qe.Z);
            if (ax.Length > 1e-9) ax.Normalize();
            Console.WriteLine();
            Console.WriteLine($"start EE {fk.eePos}  goal EE {g.eePos}  pos err {(g.eePos - fk.eePos).Length:F3}");
            Console.WriteLine($"ori err {ang * 180 / Math.PI:F1} deg about {ax}");
            Console.WriteLine($"goal angles {string.Join(", ", goal)}");
        }
    }
}
