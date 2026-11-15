using System.Windows.Media.Media3D;
using RoboticArm.Rendering;

namespace RoboticArm.Tests
{
    public sealed class BoxSplitterTests
    {
        [Theory]
        [InlineData(0, 1, 0)]
        [InlineData(0.3, 0.8, 0.5)]
        [InlineData(1, 0, 0)]
        public void Split_ProducesClosedPieces(double nx, double ny, double nz)
        {
            var (positive, negative) = BoxSplitter.Split(
                new Point3D(0, 0, 0), 0.16, new Vector3D(nx, ny, nz));

            Assert.Equal(0, BoundaryEdgeCount(positive));
            Assert.Equal(0, BoundaryEdgeCount(negative));
        }

        [Theory]
        [InlineData(0, 1, 0)]
        [InlineData(0.3, 0.8, 0.5)]
        public void Split_ConservesVolume(double nx, double ny, double nz)
        {
            var (positive, negative) = BoxSplitter.Split(
                new Point3D(0, 0, 0), 0.16, new Vector3D(nx, ny, nz));

            double expected = Math.Pow(0.32, 3);
            double positiveVolume = SignedVolume(positive);
            double negativeVolume = SignedVolume(negative);

            Assert.True(positiveVolume > 0, $"positive volume was {positiveVolume}");
            Assert.True(negativeVolume > 0, $"negative volume was {negativeVolume}");
            Assert.Equal(expected, positiveVolume + negativeVolume, 9);
        }

        [Fact]
        public void SplitPieces_RecentersHalvesOnTheirOwnCentroid()
        {
            var (positive, negative) = BoxSplitter.SplitPieces(
                new Point3D(1, 2, 3), 0.16, new Vector3D(0, 1, 0));

            Assert.Equal(0.08, positive.HalfSize.Y, 6);
            Assert.Equal(0.08, negative.HalfSize.Y, 6);
            Assert.Equal(0.16, positive.HalfSize.X, 6);
            Assert.Equal(0.16, positive.HalfSize.Z, 6);
            Assert.Equal(2.0 + 0.08, positive.Centroid.Y, 6);
            Assert.Equal(2.0 - 0.08, negative.Centroid.Y, 6);
        }

        private readonly record struct Vertex(long X, long Y, long Z);

        private static Vertex Key(Point3D p) => new(
            (long)Math.Round(p.X * 1_000_000),
            (long)Math.Round(p.Y * 1_000_000),
            (long)Math.Round(p.Z * 1_000_000));

        private static int BoundaryEdgeCount(MeshGeometry3D mesh)
        {
            var edges = new Dictionary<(Vertex, Vertex), int>();
            for (int i = 0; i < mesh.TriangleIndices.Count; i += 3)
            {
                var a = Key(mesh.Positions[mesh.TriangleIndices[i]]);
                var b = Key(mesh.Positions[mesh.TriangleIndices[i + 1]]);
                var c = Key(mesh.Positions[mesh.TriangleIndices[i + 2]]);
                AddEdge(edges, a, b);
                AddEdge(edges, b, c);
                AddEdge(edges, c, a);
            }

            return edges.Count(pair => pair.Value != 2);
        }

        private static void AddEdge(Dictionary<(Vertex, Vertex), int> edges, Vertex a, Vertex b)
        {
            var edge = Compare(a, b) <= 0 ? (a, b) : (b, a);
            edges[edge] = edges.TryGetValue(edge, out int count) ? count + 1 : 1;
        }

        private static int Compare(Vertex a, Vertex b)
        {
            int result = a.X.CompareTo(b.X);
            if (result != 0)
                return result;

            result = a.Y.CompareTo(b.Y);
            return result != 0 ? result : a.Z.CompareTo(b.Z);
        }

        private static double SignedVolume(MeshGeometry3D mesh)
        {
            double volume = 0;
            for (int i = 0; i < mesh.TriangleIndices.Count; i += 3)
            {
                var a = ToVector(mesh.Positions[mesh.TriangleIndices[i]]);
                var b = ToVector(mesh.Positions[mesh.TriangleIndices[i + 1]]);
                var c = ToVector(mesh.Positions[mesh.TriangleIndices[i + 2]]);
                volume += Vector3D.DotProduct(a, Vector3D.CrossProduct(b, c)) / 6.0;
            }

            return volume;
        }

        private static Vector3D ToVector(Point3D p) => new(p.X, p.Y, p.Z);
    }
}
