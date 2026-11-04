using System;
using System.Linq;
using System.Windows.Media.Media3D;

namespace RoboticArm.Kinematics
{
    // Simple Damped Least Squares (Levenberg-Marquardt) position-only IK solver
    public static class InverseKinematics
    {
        // Solve for joint deltas that move end-effector towards targetPosition.
        // jointAxes: array of joint axes in world coords (Vector3D[])
        // jointOrigins: array of joint origins in world coords (Point3D[])
        // angles: current angles in degrees (will be updated)
        // returns true if converged
        public static bool SolvePositionIK(Point3D target, Vector3D[] jointAxes, Point3D[] jointOrigins, double[] anglesDegrees,
            Func<double[], (Point3D eePos, Vector3D eeDir)> forwardKinematicsFunc,
            int maxIterations = 80, double tolerance = 1e-3, double lambda = 0.1)
        {
            int n = anglesDegrees.Length;
            if (jointAxes.Length != n || jointOrigins.Length != n) throw new ArgumentException("Lengths must match");

            for (int iter = 0; iter < maxIterations; iter++)
            {
                var fk = forwardKinematicsFunc(anglesDegrees);
                var ee = fk.eePos;
                var errVec = new Vector3D(target.X - ee.X, target.Y - ee.Y, target.Z - ee.Z);
                double err = errVec.Length;
                if (err < tolerance) return true;

                // Build Jacobian (3 x n)
                double[,] J = new double[3, n];
                for (int i = 0; i < n; i++)
                {
                    var axis = jointAxes[i];
                    var origin = jointOrigins[i];
                    // Jacobian column for revolute joint: axis x (ee - origin)
                    var r = ee - origin;
                    var col = Vector3D.CrossProduct(axis, r);
                    J[0, i] = col.X;
                    J[1, i] = col.Y;
                    J[2, i] = col.Z;
                }

                // Compute J * J^T (3x3) and regularize
                double[,] JJt = new double[3, 3];
                for (int a = 0; a < 3; a++) for (int b = 0; b < 3; b++)
                {
                    double sum = 0;
                    for (int k = 0; k < n; k++) sum += J[a, k] * J[b, k];
                    JJt[a, b] = sum;
                }

                // Add lambda^2 on diagonal
                for (int d = 0; d < 3; d++) JJt[d, d] += lambda * lambda;

                // Solve (JJt) * x = error vector for intermediate vector x (3x1)
                var bVec = new double[] { errVec.X, errVec.Y, errVec.Z };
                var xVec = Solve3x3(JJt, bVec);
                if (xVec == null) return false;

                // delta_theta = J^T * xVec
                double[] deltaTheta = new double[n];
                for (int i = 0; i < n; i++)
                {
                    double sum = 0;
                    for (int r = 0; r < 3; r++) sum += J[r, i] * xVec[r];
                    // small gain
                    deltaTheta[i] = sum;
                }

                // apply update (angles are degrees; convert small rad delta to degrees via factor)
                for (int i = 0; i < n; i++)
                {
                    // deltaTheta is in units of length (because J had length units); apply small scalar to make stable
                    anglesDegrees[i] += deltaTheta[i] * 0.5; // heuristic gain
                }
            }

            return false;
        }

        // Solve 3x3 linear system A x = b using Cramer's rule (or Gaussian elimination)
        private static double[] Solve3x3(double[,] A, double[] b)
        {
            double a11 = A[0, 0], a12 = A[0, 1], a13 = A[0, 2];
            double a21 = A[1, 0], a22 = A[1, 1], a23 = A[1, 2];
            double a31 = A[2, 0], a32 = A[2, 1], a33 = A[2, 2];
            double det = a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
            if (Math.Abs(det) < 1e-9) return null;
            double invDet = 1.0 / det;
            double d1 = b[0], d2 = b[1], d3 = b[2];
            double dx = d1 * (a22 * a33 - a23 * a32) - a12 * (d2 * a33 - a23 * d3) + a13 * (d2 * a32 - a22 * d3);
            double dy = a11 * (d2 * a33 - a23 * d3) - d1 * (a21 * a33 - a23 * a31) + a13 * (a21 * d3 - d2 * a31);
            double dz = a11 * (a22 * d3 - d2 * a32) - a12 * (a21 * d3 - d2 * a31) + d1 * (a21 * a32 - a22 * a31);
            return new double[] { dx * invDet, dy * invDet, dz * invDet };
        }
    }
}
