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

                // Orientation error: rotation that takes the current tool
                // orientation to the target, expressed in the same frame as
                // the analytic Jacobian below. ForwardKinematics builds
                // column-major Rodrigues matrices that WPF applies to row
                // vectors, so the matching composition is conj(current)*target
                // (the reverse order of the naive world-space formula).
                var qCurrent = QuaternionFromMatrix(eeRot);
                var qTarget = QuaternionFromMatrix(targetRot);
                var qCurrentConjugate = new Quaternion(-qCurrent.X, -qCurrent.Y, -qCurrent.Z, qCurrent.W);
                var qErr = qCurrentConjugate * qTarget;
                qErr.Normalize();
                // Take the shortest arc.
                if (qErr.W < 0)
                    qErr = new Quaternion(-qErr.X, -qErr.Y, -qErr.Z, -qErr.W);
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

                // Build 6xN Jacobian: top 3 rows linear, bottom 3 rows angular (joint axis).
                // WPF applies the FK matrices to row vectors, which reverses the
                // effective sense of a positive joint angle, so both blocks use
                // the row-vector convention (r x axis and -axis).
                double[,] J = new double[6, n];
                for (int i = 0; i < n; i++)
                {
                    var axisWorld = fk.jointAxes[i];
                    var r = ee - fk.jointOrigins[i];
                    var lin = Vector3D.CrossProduct(r, axisWorld);
                    J[0, i] = lin.X; J[1, i] = lin.Y; J[2, i] = lin.Z;
                    J[3, i] = -axisWorld.X; J[4, i] = -axisWorld.Y; J[5, i] = -axisWorld.Z;
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

                // Build error vector (6x1): position (m) then orientation (rad).
                // The orientation part is negated so it lives in the same
                // row-vector frame as the Jacobian blocks above.
                double[] err = new double[6] { errPos.X, errPos.Y, errPos.Z, -errOriVec.X, -errOriVec.Y, -errOriVec.Z };
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

                // J^T * (JJ^T)^-1 * error is the correction that moves the
                // current pose toward the target, so add it to the angles.
                // The correction is only valid for small errors, so limit
                // the per-iteration joint step instead of applying it whole.
                const double gain = 0.5;
                const double maxStepDeg = 5.0;
                for (int i = 0; i < n; i++)
                {
                    double stepDeg = delta[i] * (180.0 / Math.PI) * gain;
                    stepDeg = Math.Max(-maxStepDeg, Math.Min(maxStepDeg, stepDeg));
                    anglesDeg[i] += stepDeg;
                }

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

        // Position-only damped least-squares IK. This avoids the singular
        // 6x6 system when orientation is intentionally not being solved.
        public static bool SolvePosition(Point3D targetPos, double[] anglesDeg,
            Func<double[], (Point3D eePos, Point3D[] jointOrigins, Vector3D[] jointAxes)> forwardKinematics,
            double[] minAngles = null, double[] maxAngles = null,
            double positionTolerance = 1e-3, int maxIter = 20, double lambda = 0.12,
            Func<double[], bool> isCandidateSafe = null)
        {
            int n = anglesDeg.Length;
            for (int iter = 0; iter < maxIter; iter++)
            {
                var fk = forwardKinematics(anglesDeg);
                var error = new Vector3D(
                    targetPos.X - fk.eePos.X,
                    targetPos.Y - fk.eePos.Y,
                    targetPos.Z - fk.eePos.Z);

                if (error.Length < positionTolerance)
                    return true;

                // Turn the base toward the target before solving the
                // remaining joints. The step is rate-limited so the arm
                // cannot snap around the base within a single tick.
                if (minAngles != null && maxAngles != null)
                {
                    const double maxBaseYawStep = 2.0;
                    double desiredYaw = Math.Atan2(targetPos.Z, targetPos.X) * 180.0 / Math.PI;
                    double yawError = NormalizeAngle(desiredYaw - anglesDeg[0]);
                    if (Math.Abs(yawError) > 0.5)
                    {
                        var yawAngles = (double[])anglesDeg.Clone();
                        double yawStep = Math.Max(-maxBaseYawStep, Math.Min(maxBaseYawStep, yawError));
                        yawAngles[0] = Math.Max(minAngles[0], Math.Min(maxAngles[0], anglesDeg[0] + yawStep));
                        if (isCandidateSafe == null || isCandidateSafe(yawAngles))
                        {
                            anglesDeg[0] = yawAngles[0];
                            // The FK cache is only valid for the previous
                            // yaw, so recompute it before building J.
                            fk = forwardKinematics(anglesDeg);
                            error = new Vector3D(
                                targetPos.X - fk.eePos.X,
                                targetPos.Y - fk.eePos.Y,
                                targetPos.Z - fk.eePos.Z);
                            if (error.Length < positionTolerance)
                                return true;
                        }
                    }
                }

                // Use a numerical Jacobian so the solver does not depend on
                // the analytic axis conventions of the FK chain.
                double[,] j = new double[3, n];
                const double angleStep = 0.5;
                for (int i = 0; i < n; i++)
                {
                    var perturbedAngles = (double[])anglesDeg.Clone();
                    perturbedAngles[i] += angleStep;
                    var perturbedPosition = forwardKinematics(perturbedAngles).eePos;
                    j[0, i] = (perturbedPosition.X - fk.eePos.X) / angleStep;
                    j[1, i] = (perturbedPosition.Y - fk.eePos.Y) / angleStep;
                    j[2, i] = (perturbedPosition.Z - fk.eePos.Z) / angleStep;
                }

                double[,] jjt = new double[3, 3];
                for (int row = 0; row < 3; row++)
                {
                    for (int column = 0; column < 3; column++)
                    {
                        for (int i = 0; i < n; i++)
                            jjt[row, column] += j[row, i] * j[column, i];

                        if (row == column)
                            jjt[row, column] += lambda * lambda;
                    }
                }

                var solution = SolveLinearSystem3(jjt, new[] { error.X, error.Y, error.Z });
                if (solution == null)
                    return false;

                double[] delta = new double[n];
                for (int i = 0; i < n; i++)
                {
                    for (int row = 0; row < 3; row++)
                        delta[i] += j[row, i] * solution[row];
                }

                // Use a small backtracking line search. Near a horizontal
                // singularity, a full Jacobian update can turn the arm away
                // from the target even though the local step looks valid.
                var previousAngles = (double[])anglesDeg.Clone();
                bool accepted = false;
                for (int attempt = 0; attempt < 7; attempt++)
                {
                    Array.Copy(previousAngles, anglesDeg, n);
                    double gain = 0.8 * Math.Pow(0.5, attempt);
                    for (int i = 0; i < n; i++)
                    {
                        double value = previousAngles[i] + delta[i] * gain;
                        if (minAngles != null && maxAngles != null)
                            value = Math.Max(minAngles[i], Math.Min(maxAngles[i], value));
                        anglesDeg[i] = value;
                    }

                    var candidate = forwardKinematics(anglesDeg).eePos;
                    var candidateError = (targetPos - candidate).Length;
                    if ((isCandidateSafe == null || isCandidateSafe(anglesDeg)) && candidateError < error.Length)
                    {
                        accepted = true;
                        break;
                    }
                }

                if (!accepted)
                {
                    // Fall back to a coordinate search. This is important at
                    // a horizontal singularity, where the combined Jacobian
                    // can report no descent even though one joint can still
                    // make progress toward the waypoint.
                    Array.Copy(previousAngles, anglesDeg, n);
                    double bestError = error.Length;
                    int bestJoint = -1;
                    double bestChange = 0;
                    double[] fallbackSteps = { 0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0 };

                    for (int joint = 0; joint < n; joint++)
                    {
                        foreach (var step in fallbackSteps)
                        {
                            foreach (var direction in new[] { -1.0, 1.0 })
                            {
                                Array.Copy(previousAngles, anglesDeg, n);
                                anglesDeg[joint] += direction * step;

                                if (minAngles != null && maxAngles != null)
                                    anglesDeg[joint] = Math.Max(minAngles[joint], Math.Min(maxAngles[joint], anglesDeg[joint]));

                                var candidate = forwardKinematics(anglesDeg).eePos;
                                var candidateError = (targetPos - candidate).Length;
                                if ((isCandidateSafe == null || isCandidateSafe(anglesDeg)) && candidateError < bestError)
                                {
                                    bestError = candidateError;
                                    bestJoint = joint;
                                    bestChange = direction * step;
                                }
                            }
                        }
                    }

                    Array.Copy(previousAngles, anglesDeg, n);
                    if (bestJoint < 0)
                        return false;

                    anglesDeg[bestJoint] += bestChange;
                    if (minAngles != null && maxAngles != null)
                        anglesDeg[bestJoint] = Math.Max(minAngles[bestJoint], Math.Min(maxAngles[bestJoint], anglesDeg[bestJoint]));
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

        private static double NormalizeAngle(double degrees)
        {
            while (degrees > 180.0) degrees -= 360.0;
            while (degrees < -180.0) degrees += 360.0;
            return degrees;
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

        private static double[] SolveLinearSystem3(double[,] A, double[] b)
        {
            double[,] m = new double[3, 4];
            for (int row = 0; row < 3; row++)
            {
                for (int column = 0; column < 3; column++)
                    m[row, column] = A[row, column];
                m[row, 3] = b[row];
            }

            for (int pivot = 0; pivot < 3; pivot++)
            {
                int best = pivot;
                for (int row = pivot + 1; row < 3; row++)
                {
                    if (Math.Abs(m[row, pivot]) > Math.Abs(m[best, pivot]))
                        best = row;
                }

                if (Math.Abs(m[best, pivot]) < 1e-12)
                    return null;

                if (best != pivot)
                {
                    for (int column = pivot; column < 4; column++)
                    {
                        double value = m[pivot, column];
                        m[pivot, column] = m[best, column];
                        m[best, column] = value;
                    }
                }

                double divisor = m[pivot, pivot];
                for (int column = pivot; column < 4; column++)
                    m[pivot, column] /= divisor;

                for (int row = 0; row < 3; row++)
                {
                    if (row == pivot)
                        continue;

                    double factor = m[row, pivot];
                    for (int column = pivot; column < 4; column++)
                        m[row, column] -= factor * m[pivot, column];
                }
            }

            return new[] { m[0, 3], m[1, 3], m[2, 3] };
        }
    }
}
