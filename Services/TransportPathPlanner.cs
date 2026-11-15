using System.Windows.Media.Media3D;
using RoboticArm.Models;

namespace RoboticArm.Services
{
    public sealed record TransportPath(List<Point3D> Points, string Mode);

    // Plans a collision-free moving path for the wrist on a coarse X/Z grid
    // with a low and a high carry level. The planner prefers the cheapest
    // route, so the arm naturally dips under hanging obstacles, climbs over
    // floor blocks and detours sideways around tall pillars.
    public static class TransportPathPlanner
    {
        private const double Step = SceneConfig.PathStep;
        private const double EeHalf = SceneConfig.EeHalfExtent;
        private const double Inflate = SceneConfig.CollisionInflate;
        private const double FloorClearance = SceneConfig.FloorClearance;
        private const double BaseHeight = SceneConfig.BaseHeight;
        private const double MinZ = -SceneConfig.WorkspaceHalfDepth;
        private const double MaxZ = SceneConfig.WorkspaceHalfDepth;
        private const int LevelCount = 3;

        // Climbing or diving mid-route is slow and makes the arm pitch, so a
        // level change costs extra on top of the vertical travel itself.
        private const double LevelChangePenalty = 1.2;

        public static TransportPath Plan(
            Point3D start,
            Point3D end,
            IReadOnlyList<Obstacle> obstacles,
            Vector3D payloadOffset,
            double payloadHalf,
            double lowLevel,
            double highLevel,
            double armReach = 4.74,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints = null)
        {
            if (highLevel < lowLevel + 0.6)
                highLevel = lowLevel + 0.6;

            // Even without obstacles the route cannot cut straight across the
            // base at a low carry level, so it always goes through the grid.

            // The grid is anchored at the start pose, so the first waypoint is
            // exact and the goal snaps to at most half a cell off.
            int ixMin = -1 - (int)Math.Ceiling(0.6 / Step);
            int ixMax = 1 + (int)Math.Ceiling(0.6 / Step);
            int ixEnd = (int)Math.Round((end.X - start.X) / Step);
            ixMin = Math.Min(ixMin, ixEnd - 1);
            ixMax = Math.Max(ixMax, ixEnd + 1);

            int izMin = (int)Math.Floor((MinZ - start.Z) / Step);
            int izMax = (int)Math.Ceiling((MaxZ - start.Z) / Step);
            int izEnd = (int)Math.Round((end.Z - start.Z) / Step);

            int nx = ixMax - ixMin + 1;
            int nz = izMax - izMin + 1;
            var levels = new[] { lowLevel, (lowLevel + highLevel) * 0.5, highLevel };

            double[] g = new double[nx * nz * LevelCount];
            int[] came = new int[nx * nz * LevelCount];
            bool[] closed = new bool[nx * nz * LevelCount];
            Array.Fill(g, double.PositiveInfinity);
            Array.Fill(came, -1);

            var open = new PriorityQueue<int, double>();

            for (int level = 0; level < LevelCount; level++)
            {
                if (!IsPoseClear(start.X, levels[level], start.Z, obstacles, payloadOffset, payloadHalf, armReach, offsetHints))
                    continue;

                int index = Index(0, 0, level, ixMin, izMin, nz);
                g[index] = Math.Abs(levels[level] - start.Y);
                open.Enqueue(index, Score(index, g, start, end, ixMin, izMin, nz));
            }

            int goalNode = -1;
            while (open.TryDequeue(out int current, out double priority))
            {
                if (closed[current])
                    continue;

                if (priority > Score(current, g, start, end, ixMin, izMin, nz) + 1e-9)
                    continue;

                closed[current] = true;
                Decompose(current, ixMin, izMin, nz, out int ix, out int iz, out int level);

                // The grid is anchored at the start, so rounding can snap the
                // goal cell just outside the arm's reach even when the real end
                // point is reachable. Accept the neighbouring cells as goals;
                // the exact end point is appended to the route below.
                if (Math.Abs(ix - ixEnd) <= 1 && Math.Abs(iz - izEnd) <= 1)
                {
                    goalNode = current;
                    break;
                }

                double cx = NodeX(ix, start.X);
                double cz = NodeZ(iz, start.Z);
                double cy = levels[level];

                for (int d = 0; d < 4; d++)
                {
                    int nextIx = ix + (d == 0 ? 1 : d == 1 ? -1 : 0);
                    int nextIz = iz + (d == 2 ? 1 : d == 3 ? -1 : 0);
                    if (nextIx < ixMin || nextIx > ixMax || nextIz < izMin || nextIz > izMax)
                        continue;

                    double tx = NodeX(nextIx, start.X);
                    double tz = NodeZ(nextIz, start.Z);
                    if (!IsPoseClear(tx, cy, tz, obstacles, payloadOffset, payloadHalf, armReach, offsetHints))
                        continue;

                    Relax(Index(nextIx, nextIz, level, ixMin, izMin, nz), current, g[current] + Step,
                        g, came, open, start, end, ixMin, izMin, nz);
                }

                // Only adjacent levels connect, so a climb cannot tunnel through
                // a blocked intermediate level.
                for (int step = -1; step <= 1; step += 2)
                {
                    int other = level + step;
                    if (other < 0 || other >= LevelCount)
                        continue;

                    double oy = levels[other];
                    if (IsPoseClear(cx, oy, cz, obstacles, payloadOffset, payloadHalf, armReach, offsetHints))
                    {
                        Relax(Index(ix, iz, other, ixMin, izMin, nz), current,
                            g[current] + Math.Abs(oy - cy) + LevelChangePenalty,
                            g, came, open, start, end, ixMin, izMin, nz);
                    }
                }
            }

            if (goalNode < 0)
            {
                // No grid route exists (e.g. the grown payload envelope plugs
                // every corridor). Callers must not drive this leg - a made-up
                // straight line can demand poses beyond the arm's reach.
                return new TransportPath(new List<Point3D> { start, end }, "unreachable");
            }

            var raw = new List<Point3D>();
            for (int node = goalNode; node >= 0; node = came[node])
            {
                Decompose(node, ixMin, izMin, nz, out int ix, out int iz, out int level);
                raw.Add(new Point3D(NodeX(ix, start.X), levels[level], NodeZ(iz, start.Z)));
            }

            raw.Reverse();
            raw[^1] = new Point3D(end.X, raw[^1].Y, end.Z);

            var points = Simplify(raw);
            points.Add(end);
            points = Shortcut(points, obstacles, payloadOffset, payloadHalf, armReach, offsetHints);

            return new TransportPath(points, DescribeMode(raw, start, end, lowLevel, highLevel, obstacles, payloadOffset, payloadHalf));
        }

        private static List<Point3D> Simplify(List<Point3D> points)
        {
            if (points.Count < 3)
                return points;

            var result = new List<Point3D> { points[0] };
            for (int i = 1; i < points.Count - 1; i++)
            {
                var first = points[i] - points[i - 1];
                var second = points[i + 1] - points[i];
                if (Vector3D.CrossProduct(first, second).Length > 1e-6)
                    result.Add(points[i]);
            }

            result.Add(points[^1]);
            return result;
        }

        // The grid only moves cardinally, so its paths staircase around every
        // corner. Cutting a corner is allowed whenever the straight segment
        // stays clear, which removes most of the zig-zag the arm would
        // otherwise track waypoint by waypoint.
        private static List<Point3D> Shortcut(
            List<Point3D> points,
            IReadOnlyList<Obstacle> obstacles,
            Vector3D payloadOffset,
            double payloadHalf,
            double armReach,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints)
        {
            if (points.Count < 3)
                return points;

            var result = new List<Point3D> { points[0] };
            int anchor = 0;
            for (int i = 2; i < points.Count; i++)
            {
                if (SegmentClear(points[anchor], points[i], obstacles, payloadOffset, payloadHalf, armReach, offsetHints))
                    continue;

                result.Add(points[i - 1]);
                anchor = i - 1;
            }

            result.Add(points[^1]);
            return result;
        }

        private static bool SegmentClear(
            Point3D from, Point3D to,
            IReadOnlyList<Obstacle> obstacles,
            Vector3D payloadOffset,
            double payloadHalf,
            double armReach,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints)
        {
            int samples = Math.Max(2, (int)Math.Ceiling((to - from).Length / 0.07));
            for (int i = 1; i < samples; i++)
            {
                var p = from + (to - from) * ((double)i / samples);
                if (!IsPoseClear(p.X, p.Y, p.Z, obstacles, payloadOffset, payloadHalf, armReach, offsetHints))
                    return false;
            }

            return true;
        }

        private static string DescribeMode(
            List<Point3D> raw, Point3D start, Point3D end,
            double lowLevel, double highLevel,
            IReadOnlyList<Obstacle> obstacles,
            Vector3D payloadOffset, double payloadHalf)
        {
            if (obstacles.Count == 0)
                return "straight";

            bool passesUnder = false;
            bool passesOver = false;
            double detour = 0;
            double dx = end.X - start.X;
            double dz = end.Z - start.Z;

            foreach (var point in raw)
            {
                double t = Math.Abs(dx) < 1e-6 ? 0 : (point.X - start.X) / dx;
                double lineZ = start.Z + t * dz;
                detour = Math.Max(detour, Math.Abs(point.Z - lineZ));

                foreach (var obstacle in obstacles)
                {
                    double bottom = obstacle.Center.Y - obstacle.HalfSize.Y;
                    double top = obstacle.Center.Y + obstacle.HalfSize.Y;
                    double margin = payloadHalf + 0.3;

                    if (point.X < obstacle.Center.X - obstacle.HalfSize.X - margin ||
                        point.X > obstacle.Center.X + obstacle.HalfSize.X + margin ||
                        point.Z < obstacle.Center.Z - obstacle.HalfSize.Z - margin ||
                        point.Z > obstacle.Center.Z + obstacle.HalfSize.Z + margin)
                    {
                        continue;
                    }

                    if (point.Y + payloadOffset.Y + payloadHalf < bottom && point.Y - EeHalf < bottom)
                        passesUnder = true;
                    else if (point.Y - EeHalf > top)
                        passesOver = true;
                }
            }

            var modes = new List<string>();
            if (passesOver)
                modes.Add("over");
            if (passesUnder)
                modes.Add("under");
            if (detour > 0.45)
                modes.Add("around");
            if (modes.Count == 0)
            {
                if (Math.Abs(raw[0].Y - lowLevel) < 0.01)
                    modes.Add("under");
                else if (Math.Abs(raw[0].Y - highLevel) < 0.01)
                    modes.Add("over");
            }

            return modes.Count == 0 ? "straight" : string.Join("+", modes);
        }

        private static bool IsPoseClear(
            double x, double y, double z,
            IReadOnlyList<Obstacle> obstacles,
            Vector3D payloadOffset,
            double payloadHalf,
            double armReach,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>>? offsetHints)
        {
            if (y - EeHalf < FloorClearance)
                return false;

            double radius = Math.Sqrt(x * x + z * z);

            // Close to its base the arm cannot fold down: low and middle carry
            // levels only exist near the outer part of the workspace.
            if (radius < MinCarryRadius(y))
                return false;

            // The opposite holds up high: the carry level must stay within the
            // arm's reach at this horizontal distance. The route therefore
            // climbs over obstacles only once it has retracted far enough.
            double maxY = BaseHeight + Math.Sqrt(Math.Max(0.0, armReach * armReach - radius * radius));
            if (y > maxY)
                return false;

            var offset = offsetHints?.Count > 0 ? NearestOffset(x, z, payloadOffset, offsetHints) : payloadOffset;
            double px = x + offset.X;
            double py = y + offset.Y;
            double pz = z + offset.Z;

            foreach (var obstacle in obstacles)
            {
                if (Overlaps(x, y, z, EeHalf, obstacle) ||
                    Overlaps(px, py, pz, payloadHalf, obstacle))
                {
                    return false;
                }
            }

            return true;
        }

        // The payload hangs off the tool, so its world offset depends on the
        // wrist orientation along the route, not on the pose the arm happened
        // to be in when planning started. The caller feeds the offsets it
        // measured along a previous pose plan; nodes far from that route fall
        // back to the constant planning offset.
        private const double HintRadius = 0.6;

        private static Vector3D NearestOffset(
            double x, double z, Vector3D fallback,
            IReadOnlyList<KeyValuePair<Point3D, Vector3D>> hints)
        {
            double best = HintRadius * HintRadius;
            Vector3D offset = fallback;
            for (int i = 0; i < hints.Count; i++)
            {
                double dx = hints[i].Key.X - x;
                double dz = hints[i].Key.Z - z;
                double d2 = dx * dx + dz * dz;
                if (d2 < best)
                {
                    best = d2;
                    offset = hints[i].Value;
                }
            }

            return offset;
        }

        // Inner workspace boundary measured from the joint limits: the arm
        // needs more room around the base axis the lower it carries. Up high
        // the opposite fold limit applies too - the shoulder, elbow and wrist
        // cannot all fold enough to bring the tool back over the base.
        private static double MinCarryRadius(double y)
        {
            if (y >= 2.35)
                return 1.20;
            if (y >= 2.0)
                return 1.75;
            if (y >= 1.45)
                return 2.05;
            return 2.40;
        }

        private static bool Overlaps(double x, double y, double z, double half, Obstacle obstacle)
        {
            var center = obstacle.Center;
            var size = obstacle.HalfSize;
            return x + half > center.X - size.X - Inflate &&
                   x - half < center.X + size.X + Inflate &&
                   y + half > center.Y - size.Y - Inflate &&
                   y - half < center.Y + size.Y + Inflate &&
                   z + half > center.Z - size.Z - Inflate &&
                   z - half < center.Z + size.Z + Inflate;
        }

        private static void Relax(
            int node, int from, double cost, double[] g, int[] came,
            PriorityQueue<int, double> open, Point3D start, Point3D end, int ixMin, int izMin, int nz)
        {
            if (cost >= g[node])
                return;

            g[node] = cost;
            came[node] = from;
            open.Enqueue(node, Score(node, g, start, end, ixMin, izMin, nz));
        }

        private static double Score(int node, double[] g, Point3D start, Point3D end, int ixMin, int izMin, int nz)
        {
            Decompose(node, ixMin, izMin, nz, out int ix, out int iz, out _);
            double dx = end.X - NodeX(ix, start.X);
            double dz = end.Z - NodeZ(iz, start.Z);
            return g[node] + Math.Sqrt(dx * dx + dz * dz);
        }

        private static int Index(int ix, int iz, int level, int ixMin, int izMin, int nz)
            => (((ix - ixMin) * nz) + (iz - izMin)) * LevelCount + level;

        private static void Decompose(int index, int ixMin, int izMin, int nz, out int ix, out int iz, out int level)
        {
            level = index % LevelCount;
            int cell = index / LevelCount;
            iz = cell % nz + izMin;
            ix = cell / nz + ixMin;
        }

        private static double NodeX(int ix, double startX) => startX + ix * Step;

        private static double NodeZ(int iz, double startZ) => startZ + iz * Step;
    }
}
