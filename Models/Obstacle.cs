using System.Windows.Media.Media3D;

namespace RoboticArm.Models
{
    public sealed class Obstacle
    {
        public Point3D Center { get; init; }
        public Vector3D HalfSize { get; init; }
    }
}
