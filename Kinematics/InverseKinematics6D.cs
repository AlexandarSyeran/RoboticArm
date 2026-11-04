using System;
using System.Windows.Media.Media3D;

namespace RoboticArm.Kinematics
{
    // Damped Least Squares 6D IK solver (position + orientation)
    public static class InverseKinematics6D
    {
        // forwardKinematics: given angles (degrees) returns (eePos, eeRotationMatrix, jointOrigins[], jointAxes[])
        // jointAxes are expressed in world coordinates
        public static bool Solve6D(Point3D targetPos, Matrix3D targetRot, double[] anglesDeg,
            Func<double[], (Point3D eePos, Matrix3D eeRot, Point3D[] jointOrigins, Vector3D[] jointAxes)> forwardKinematics,
            double[] minAngles = null, double[] maxAngles = null,
            bool useOrientation = true,
            double positionTolerance = 1e-3, double orientationToleranceDeg = 1e-2,
            int maxIter = 120, double lambda = 0.1)
        {
            int n = anglesDeg.Length;
            for (int iter = 0; iter < maxIter; iter++)
            {
                var fk = forwardKinematics(anglesDeg);
                var ee = fk.eePos;
                var eeRot = fk.eeRot;

                // position error
                var errPos = new Vector3D(targetPos.X - ee.X, targetPos.Y - ee.Y, targetPos.Z - ee.Z);
                double posLen = errPos.Length;

                // orientation error: use quaternion error -> axis-angle vector
                var qCurrent = QuaternionFromMatrix(eeRot);
                var qTarget = QuaternionFromMatrix(targetRot);
                var qErr = qTarget * qCurrent; // note: Matrix->Quaternion gives rotation from local to world; multiply gives error
                qErr.Normalize();
                // convert qErr to axis-angle: angle = 2*acos(w), axis = v / sin(angle/2)
                var axis = new Vector3D(qErr.X, qErr.Y, qErr.Z);
                double angle = 2.0 * Math.Acos(Math.Max(-1.0, Math.Min(1.0, qErr.W))); // radians
                Vector3D errOriVec;
                if (Math.Abs(angle) < 1e-6 || axis.Length < 1e-6)
                {
                    errOriVec = new Vector3D(0, 0, 0);
                }
                else
                {
                    axis.Normalize();
                    errOriVec = axis * angle; // rotation vector (radians)
                }
                double oriLen = errOriVec.Length;

                if (!useOrientation)
                {
                    if (posLen < positionTolerance) return true;
                }
                else
                {
                    if (posLen < positionTolerance && oriLen < (orientationToleranceDeg * Math.PI / 180.0)) return true;
                }

                // Build 6xN Jacobian: top 3 rows linear, bottom 3 rows angular (joint axis)
                double[,] J = new double[6, n];
                for (int i = 0; i < n; i++)
                {
                    var axisWorld = fk.jointAxes[i];
                    var r = ee - fk.jointOrigins[i];
                    var lin = Vector3D.CrossProduct(axisWorld, r);
                    J[0, i] = lin.X; J[1, i] = lin.Y; J[2, i] = lin.Z;
                    J[3, i] = axisWorld.X; J[4, i] = axisWorld.Y; J[5, i] = axisWorld.Z;
                }

                // Build JJ^T (6x6) = J * J^T
                double[,] JJt = new double[6, 6];
                for (int a = 0; a < 6; a++) for (int b = 0; b < 6; b++)
                {
                    double sum = 0;
                    for (int k = 0; k < n; k++) sum += J[a, k] * J[b, k];
                    JJt[a, b] = sum;
                }

                // Regularize diagonal
                for (int d = 0; d < 6; d++) JJt[d, d] += lambda * lambda;

                // Build error vector (6x1): position (m) then orientation (rad)
                double[] err = new double[6] { errPos.X, errPos.Y, errPos.Z, errOriVec.X, errOriVec.Y, errOriVec.Z };
                if (!useOrientation)
                {
                    // zero out orientation rows and columns in JJt and J to effectively ignore orientation
                    for (int i = 3; i < 6; i++) { err[i] = 0; for (int j = 0; j < 6; j++) JJt[i, j] = JJt[j, i] = 0; JJt[i, i] = lambda * lambda; }
                }

                // Solve JJt * x = err  (6x6)
                var x = SolveLinearSystem6(JJt, err);
                if (x == null) return false;

                // deltaTheta = J^T * x
                double[] delta = new double[n];
                for (int i = 0; i < n; i++)
                {
                    double s = 0;
                    for (int r2 = 0; r2 < 6; r2++) s += J[r2, i] * x[r2];
                    delta[i] = s;
                }

                // Apply update (delta is in mixed units; scale small). We'll apply projection/clamping each iteration.
                for (int i = 0; i < n; i++) anglesDeg[i] += delta[i] * (180.0 / Math.PI) * 0.6; // convert rad->deg and apply gain

                // enforce joint limits if provided
                if (minAngles != null && maxAngles != null)
                {
                    for (int i = 0; i < n; i++)
                    {
                        if (anglesDeg[i] < minAngles[i]) anglesDeg[i] = minAngles[i];
                        if (anglesDeg[i] > maxAngles[i]) anglesDeg[i] = maxAngles[i];
                    }
                }
            }

            return false;
        }

        // Convert rotation matrix to quaternion
        private static Quaternion QuaternionFromMatrix(Matrix3D m)
        {
            // Convert rotation matrix to quaternion (assuming no scaling)
            double trace = m.M11 + m.M22 + m.M33;
            double w, x, y, z;
            if (trace > 0)
            {
                double s = 0.5 / Math.Sqrt(trace + 1.0);
                w = 0.25 / s;
                x = (m.M32 - m.M23) * s;
                y = (m.M13 - m.M31) * s;
                z = (m.M21 - m.M12) * s;
            }
            else
            {
                if (m.M11 > m.M22 && m.M11 > m.M33)
                {
                    double s = 2.0 * Math.Sqrt(1.0 + m.M11 - m.M22 - m.M33);
                    w = (m.M32 - m.M23) / s;
                    x = 0.25 * s;
                    y = (m.M12 + m.M21) / s;
                    z = (m.M13 + m.M31) / s;
                }
                else if (m.M22 > m.M33)
                {
                    double s = 2.0 * Math.Sqrt(1.0 + m.M22 - m.M11 - m.M33);
                    w = (m.M13 - m.M31) / s;
                    x = (m.M12 + m.M21) / s;
                    y = 0.25 * s;
                    z = (m.M23 + m.M32) / s;
                }
                else
                {
                    double s = 2.0 * Math.Sqrt(1.0 + m.M33 - m.M11 - m.M22);
                    w = (m.M21 - m.M12) / s;
                    x = (m.M13 + m.M31) / s;
                    y = (m.M23 + m.M32) / s;
                    z = 0.25 * s;
                }
            }
            var q = new Quaternion(x, y, z, w);
            q.Normalize();
            return q;
        }

        // Solve 6x6 linear system A x = b using Gaussian elimination (simple)
        private static double[] SolveLinearSystem6(double[,] A, double[] b)
        {
            int n = 6;
            double[,] M = new double[n, n + 1];
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++) M[i, j] = A[i, j];
                M[i, n] = b[i];
            }
            // Gaussian elimination
            for (int i = 0; i < n; i++)
            {
                // pivot
                int piv = i;
                for (int k = i + 1; k < n; k++) if (Math.Abs(M[k, i]) > Math.Abs(M[piv, i])) piv = k;
                if (Math.Abs(M[piv, i]) < 1e-12) return null;
                if (piv != i) for (int j = i; j <= n; j++) { var tmp = M[i, j]; M[i, j] = M[piv, j]; M[piv, j] = tmp; }
                double div = M[i, i];
                for (int j = i; j <= n; j++) M[i, j] /= div;
                for (int r = 0; r < n; r++) if (r != i)
                {
                    double factor = M[r, i];
                    if (Math.Abs(factor) < 1e-15) continue;
                    for (int j = i; j <= n; j++) M[r, j] -= factor * M[i, j];
                }
            }
            var x = new double[n];
            for (int i = 0; i < n; i++) x[i] = M[i, n];
            return x;
        }
    }
}
