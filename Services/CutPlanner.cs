using System.Windows.Media.Media3D;
using RoboticArm.Models;
using RoboticArm.Rendering;

namespace RoboticArm.Services
{
    // Turns a cut object into two independent items, so the automation sees
    // them like any other object on the pickup zone.
    public static class CutPlanner
    {
        public const double CutHalfSize = 0.16;
        private const double OffcutSlide = 0.16;

        public static (AutomationItem Keep, AutomationItem Offcut) Split(AutomationItem item, Vector3D normal)
        {
            var (keep, offcut) = BoxSplitter.SplitPieces(item.Position, CutHalfSize, normal);

            // The upper half slides aside so both pieces are clearly visible.
            var slide = Vector3D.CrossProduct(normal, new Vector3D(0, 1, 0));
            if (slide.LengthSquared < 1e-8)
                slide = new Vector3D(1, 0, 0);
            slide.Normalize();
            var offcutPosition = offcut.Centroid + slide * OffcutSlide + new Vector3D(0, 0.03, 0);

            var keepItem = new AutomationItem
            {
                StartPosition = keep.Centroid,
                Position = keep.Centroid,
                PaletteIndex = item.PaletteIndex,
                HalfSize = keep.HalfSize,
                CutNormal = normal,
                CutCenter = item.Position,
                CutSide = 0
            };

            var offcutItem = new AutomationItem
            {
                StartPosition = offcut.Centroid,
                Position = offcutPosition,
                PaletteIndex = item.PaletteIndex,
                HalfSize = offcut.HalfSize,
                CutNormal = normal,
                CutCenter = item.Position,
                CutSide = 1
            };

            return (keepItem, offcutItem);
        }
    }
}
