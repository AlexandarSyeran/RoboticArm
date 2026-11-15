using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

namespace RoboticArm.Rendering
{
    // Splits a cube into two closed halves along a plane. The plane normal
    // comes from the actual tool direction at the moment of the cut, so the
    // pieces the arm produces follow its pose.
    public static class BoxSplitter
    {
        public readonly record struct CutPiece(
            MeshGeometry3D Mesh,
            Point3D Centroid,
            Vector3D HalfSize);

        // Returns both halves with their meshes centered on their own centroid,
        // ready to be placed at the piece position.
        public static (CutPiece positive, CutPiece negative) SplitPieces(
            Point3D center, double half, Vector3D normal)
        {
            var (positive, negative) = Split(center, half, normal);
            return (Recenter(positive), Recenter(negative));
        }

        private static CutPiece Recenter(MeshGeometry3D mesh)
        {
            var min = new Point3D(double.MaxValue, double.MaxValue, double.MaxValue);
            var max = new Point3D(double.MinValue, double.MinValue, double.MinValue);
            foreach (var p in mesh.Positions)
            {
                min = new Point3D(Math.Min(min.X, p.X), Math.Min(min.Y, p.Y), Math.Min(min.Z, p.Z));
                max = new Point3D(Math.Max(max.X, p.X), Math.Max(max.Y, p.Y), Math.Max(max.Z, p.Z));
            }

            var centroid = new Point3D(
                (min.X + max.X) / 2.0,
                (min.Y + max.Y) / 2.0,
                (min.Z + max.Z) / 2.0);
            var halfSize = new Vector3D(
                (max.X - min.X) / 2.0,
                (max.Y - min.Y) / 2.0,
                (max.Z - min.Z) / 2.0);

            for (int i = 0; i < mesh.Positions.Count; i++)
            {
                var p = mesh.Positions[i];
                mesh.Positions[i] = new Point3D(p.X - centroid.X, p.Y - centroid.Y, p.Z - centroid.Z);
            }

            return new CutPiece(mesh, centroid, halfSize);
        }

        public static (MeshGeometry3D positive, MeshGeometry3D negative) Split(
            Point3D center, double half, Vector3D normal)
        {
            if (normal.Length < 1e-9)
                normal = new Vector3D(0, 1, 0);
            normal.Normalize();

            var corners = new Point3D[8];
            int k = 0;
            for (int x = -1; x <= 1; x += 2)
            {
                for (int y = -1; y <= 1; y += 2)
                {
                    for (int z = -1; z <= 1; z += 2)
                    {
                        corners[k++] = new Point3D(
                            center.X + x * half,
                            center.Y + y * half,
                            center.Z + z * half);
                    }
                }
            }

            int C(int x, int y, int z) => (x > 0 ? 4 : 0) + (y > 0 ? 2 : 0) + (z > 0 ? 1 : 0);

            var quads = new[]
            {
                (C(-1, -1, -1), C(-1, -1, 1), C(-1, 1, 1), C(-1, 1, -1)),
                (C(1, -1, -1), C(1, 1, -1), C(1, 1, 1), C(1, -1, 1)),
                (C(-1, -1, -1), C(1, -1, -1), C(1, -1, 1), C(-1, -1, 1)),
                (C(-1, 1, -1), C(-1, 1, 1), C(1, 1, 1), C(1, 1, -1)),
                (C(-1, -1, -1), C(-1, 1, -1), C(1, 1, -1), C(1, -1, -1)),
                (C(-1, -1, 1), C(1, -1, 1), C(1, 1, 1), C(-1, 1, 1))
            };

            var positive = new MeshGeometry3D();
            var negative = new MeshGeometry3D();
            var capPoints = new List<Point3D>();
            var tri = new Point3D[3];
            var dist = new double[3];

            foreach (var quad in quads)
            {
                SplitTriangle(corners[quad.Item1], corners[quad.Item2], corners[quad.Item3]);
                SplitTriangle(corners[quad.Item1], corners[quad.Item3], corners[quad.Item4]);
            }

            // The cross-section of a convex box is a convex polygon, so the
            // intersection points ordered around their centroid give the cap.
            if (capPoints.Count >= 3)
            {
                var unique = new List<Point3D>();
                foreach (var p in capPoints)
                {
                    bool duplicate = false;
                    foreach (var q in unique)
                    {
                        if ((p - q).LengthSquared < 1e-12)
                        {
                            duplicate = true;
                            break;
                        }
                    }

                    if (!duplicate)
                        unique.Add(p);
                }

                AddCap(unique, center, normal, positive, flip: false);
                AddCap(unique, center, normal, negative, flip: true);
            }

            return (positive, negative);

            void SplitTriangle(Point3D a, Point3D b, Point3D c)
            {
                tri[0] = a; tri[1] = b; tri[2] = c;
                for (int i = 0; i < 3; i++)
                    dist[i] = Vector3D.DotProduct(tri[i] - center, normal);

                if (dist[0] >= 0 && dist[1] >= 0 && dist[2] >= 0)
                {
                    AddTriangle(positive, a, b, c);
                    return;
                }

                if (dist[0] <= 0 && dist[1] <= 0 && dist[2] <= 0)
                {
                    AddTriangle(negative, a, b, c);
                    return;
                }

                var pos = new List<Point3D>(4);
                var neg = new List<Point3D>(4);
                for (int i = 0; i < 3; i++)
                {
                    var from = tri[i];
                    var to = tri[(i + 1) % 3];
                    double dFrom = dist[i];
                    double dTo = dist[(i + 1) % 3];

                    if (dFrom >= 0)
                        pos.Add(from);
                    if (dFrom <= 0)
                        neg.Add(from);

                    if ((dFrom > 0 && dTo < 0) || (dFrom < 0 && dTo > 0))
                    {
                        var hit = from + (to - from) * (dFrom / (dFrom - dTo));
                        pos.Add(hit);
                        neg.Add(hit);
                        capPoints.Add(hit);
                    }
                }

                FanTriangulate(positive, pos);
                FanTriangulate(negative, neg);
            }
        }

        private static void FanTriangulate(MeshGeometry3D mesh, List<Point3D> polygon)
        {
            for (int i = 1; i + 1 < polygon.Count; i++)
                AddTriangle(mesh, polygon[0], polygon[i], polygon[i + 1]);
        }

        private static void AddCap(
            List<Point3D> points, Point3D center, Vector3D normal, MeshGeometry3D mesh, bool flip)
        {
            var centroid = new Point3D(0, 0, 0);
            foreach (var p in points)
                centroid = new Point3D(centroid.X + p.X, centroid.Y + p.Y, centroid.Z + p.Z);
            centroid = new Point3D(centroid.X / points.Count, centroid.Y / points.Count, centroid.Z / points.Count);

            var u = Vector3D.CrossProduct(normal, new Vector3D(1, 0, 0));
            if (u.LengthSquared < 1e-9)
                u = Vector3D.CrossProduct(normal, new Vector3D(0, 1, 0));
            u.Normalize();
            var v = Vector3D.CrossProduct(normal, u);
            v.Normalize();

            var ordered = new List<Point3D>(points);
            var key = new Dictionary<Point3D, double>();
            foreach (var p in ordered)
            {
                var rel = p - centroid;
                key[p] = Math.Atan2(Vector3D.DotProduct(rel, v), Vector3D.DotProduct(rel, u));
            }

            ordered.Sort((left, right) => key[left].CompareTo(key[right]));
            for (int i = 0; i < ordered.Count; i++)
            {
                var a = ordered[i];
                var b = ordered[(i + 1) % ordered.Count];
                if (flip)
                    AddTriangle(mesh, centroid, b, a);
                else
                    AddTriangle(mesh, centroid, a, b);
            }
        }

        private static void AddTriangle(MeshGeometry3D mesh, Point3D a, Point3D b, Point3D c)
        {
            var normal = Vector3D.CrossProduct(b - a, c - a);
            if (normal.LengthSquared < 1e-12)
                return;

            normal.Normalize();
            int start = mesh.Positions.Count;
            mesh.Positions.Add(a);
            mesh.Positions.Add(b);
            mesh.Positions.Add(c);
            mesh.Normals.Add(normal);
            mesh.Normals.Add(normal);
            mesh.Normals.Add(normal);
            mesh.TriangleIndices.Add(start);
            mesh.TriangleIndices.Add(start + 1);
            mesh.TriangleIndices.Add(start + 2);
        }
    }
}
