using System;
using System.Windows.Media.Media3D;

namespace RoboticArm.Kinematics
{
    public static class ForwardKinematics
    {
        // Build a homogeneous transform: rotation about axis (unit) by degrees, then translation
        public static Matrix3D RotateAxisAngle(Vector3D axis, double degrees)
        {
            var a = axis;
            a.Normalize();
            double rad = degrees * Math.PI / 180.0;
            double c = Math.Cos(rad), s = Math.Sin(rad);
            double x = a.X, y = a.Y, z = a.Z;
            // Rodrigues rotation matrix
            var m = new Matrix3D(
                c + x * x * (1 - c), x * y * (1 - c) - z * s, x * z * (1 - c) + y * s, 0,
                y * x * (1 - c) + z * s, c + y * y * (1 - c), y * z * (1 - c) - x * s, 0,
                z * x * (1 - c) - y * s, z * y * (1 - c) + x * s, c + z * z * (1 - c), 0,
                0, 0, 0, 1);
            return m;
        }

        public static Matrix3D Translate(double x, double y, double z)
        {
            var m = Matrix3D.Identity;
            m.OffsetX = x; m.OffsetY = y; m.OffsetZ = z;
            return m;
        }

        // Simple 2D planar IK for two-link arm in X-Y plane: returns (theta1, theta2) in degrees or null if unreachable
        public static (double theta1, double theta2)? TwoLinkPlanarIK(double x, double y, double l1, double l2)
        {
            double r = Math.Sqrt(x * x + y * y);
            if (r > l1 + l2 || r < Math.Abs(l1 - l2)) return null;
            double cos2 = (r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2);
            cos2 = Math.Max(-1.0, Math.Min(1.0, cos2));
            double s2 = Math.Sqrt(1 - cos2 * cos2);
            double theta2 = Math.Atan2(s2, cos2); // elbow angle
            double phi = Math.Atan2(y, x);
            double psi = Math.Atan2(l2 * Math.Sin(theta2), l1 + l2 * Math.Cos(theta2));
            double theta1 = phi - psi;
            return (theta1 * 180.0 / Math.PI, theta2 * 180.0 / Math.PI);
        }
    }
}
