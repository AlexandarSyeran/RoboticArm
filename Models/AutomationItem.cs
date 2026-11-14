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

        // Half extents of the object. A whole block uses the default 0.16 cube;
        // cut pieces carry the extents of their own half.
        public Vector3D HalfSize { get; init; } = new(0.16, 0.16, 0.16);

        // Set when this object is a piece produced by a cut. The renderer
        // rebuilds the piece from the original block and this plane.
        public Vector3D? CutNormal { get; init; }
        public Point3D CutCenter { get; init; }
        public int CutSide { get; init; }
    }
}
