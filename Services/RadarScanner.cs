using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    public readonly record struct RadarDetection(
        AutomationItem Item,
        Point3D Position,
        Vector3D HalfSize,
        double Distance);

    // Stand-in for the workspace sensor: reports the objects resting in a
    // zone with their measured position and size, nearest first.
    public sealed class RadarScanner
    {
        public const double ZoneHalfExtent = 0.9;

        private readonly IReadOnlyList<AutomationItem> _items;
        private readonly Point3D _zoneCenter;
        private readonly bool _includeCompleted;

        public RadarScanner(IReadOnlyList<AutomationItem> items, Point3D zoneCenter, bool includeCompleted = false)
        {
            _items = items;
            _zoneCenter = zoneCenter;
            _includeCompleted = includeCompleted;
        }

        public List<RadarDetection> Scan()
        {
            var found = new List<RadarDetection>();
            foreach (var item in _items)
            {
                if (item.Attached || (!_includeCompleted && item.Completed))
                    continue;

                if (Math.Abs(item.Position.X - _zoneCenter.X) > ZoneHalfExtent ||
                    Math.Abs(item.Position.Z - _zoneCenter.Z) > ZoneHalfExtent ||
                    item.Position.Y < 0.15 ||
                    item.Position.Y > 1.2)
                    continue;

                found.Add(new RadarDetection(
                    item,
                    item.Position,
                    item.HalfSize,
                    (item.Position - _zoneCenter).Length));
            }

            found.Sort((a, b) => a.Distance.CompareTo(b.Distance));
            return found;
        }
    }
}
