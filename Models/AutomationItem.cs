using System.Windows.Media.Media3D;

namespace RoboticArm.Models
{
    public sealed class AutomationItem
    {
        public Point3D StartPosition { get; init; }
        public Point3D Position { get; set; }
        public bool Completed { get; set; }
        public bool Attached { get; set; }
        public int PaletteIndex { get; init; }
    }
}
