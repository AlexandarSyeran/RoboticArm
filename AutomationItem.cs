using System.Windows.Media.Media3D;

namespace RoboticArm
{
    public sealed class AutomationItem
    {
        public GeometryModel3D Model { get; init; }
        public Point3D StartPosition { get; init; }
        public bool Completed { get; set; }
    }
}
