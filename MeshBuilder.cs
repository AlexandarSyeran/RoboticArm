using System;
using System.Windows.Media;
using System.Windows.Media.Media3D;

/// <summary>
/// Simple mesh building helpers for WPF 3D primitives.
/// </summary>
public static class MeshBuilder
{
    // Create a box mesh centered at the origin with given dimensions.
    public static MeshGeometry3D CreateBox(double lengthX, double heightY, double depthZ)
    {
        var hx = lengthX / 2.0;
        var hy = heightY / 2.0;
        var hz = depthZ / 2.0;
        var m = new MeshGeometry3D();

        // 8 corners, order: (-x,-y,-z), (x,-y,-z), (x,y,-z), (-x,y,-z), (-x,-y,z), (x,-y,z), (x,y,z), (-x,y,z)
        m.Positions.Add(new Point3D(-hx, -hy, -hz)); // 0
        m.Positions.Add(new Point3D(hx, -hy, -hz));  // 1
        m.Positions.Add(new Point3D(hx, hy, -hz));   // 2
        m.Positions.Add(new Point3D(-hx, hy, -hz));  // 3
        m.Positions.Add(new Point3D(-hx, -hy, hz));  // 4
        m.Positions.Add(new Point3D(hx, -hy, hz));   // 5
        m.Positions.Add(new Point3D(hx, hy, hz));    // 6
        m.Positions.Add(new Point3D(-hx, hy, hz));   // 7

        // helper
        void T(int a, int b, int c)
        {
            m.TriangleIndices.Add(a);
            m.TriangleIndices.Add(b);
            m.TriangleIndices.Add(c);
        }

        // faces, each face has two triangles
        T(0, 2, 1); T(0, 3, 2); // front
        T(4, 5, 6); T(4, 6, 7); // back
        T(0, 4, 7); T(0, 7, 3); // left
        T(1, 2, 6); T(1, 6, 5); // right
        T(3, 7, 6); T(3, 6, 2); // top
        T(0, 1, 5); T(0, 5, 4); // bottom

        // compute simple face normals
        m.Normals = CalculateNormals(m);

        // texture coords (box mapping)
        m.TextureCoordinates.Add(new System.Windows.Point(0, 1));
        m.TextureCoordinates.Add(new System.Windows.Point(1, 1));
        m.TextureCoordinates.Add(new System.Windows.Point(1, 0));
        m.TextureCoordinates.Add(new System.Windows.Point(0, 0));
        m.TextureCoordinates.Add(new System.Windows.Point(0, 1));
        m.TextureCoordinates.Add(new System.Windows.Point(1, 1));
        m.TextureCoordinates.Add(new System.Windows.Point(1, 0));
        m.TextureCoordinates.Add(new System.Windows.Point(0, 0));

        return m;
    }
    // Create a sphere mesh centered at the origin with given radius, slices, and stacks.

    public static MeshGeometry3D CreateSphere(double radius, int slices = 16, int stacks = 12)
    {
        var m = new MeshGeometry3D();
        for (int stack = 0; stack <= stacks; stack++)
        {
            double phi = Math.PI * stack / stacks;
            double y = Math.Cos(phi);
            double r = Math.Sin(phi);
            for (int slice = 0; slice <= slices; slice++)
            {
                double theta = 2.0 * Math.PI * slice / slices;
                double x = r * Math.Cos(theta);
                double z = r * Math.Sin(theta);
                var p = new Point3D(x * radius, y * radius, z * radius);
                m.Positions.Add(p);
                var n = new Vector3D(x, y, z);
                n.Normalize();
                m.Normals.Add(n);
                m.TextureCoordinates.Add(new System.Windows.Point((double)slice / slices, (double)stack / stacks));
            }
        }
        for (int stack = 0; stack < stacks; stack++)
        {
            for (int slice = 0; slice < slices; slice++)
            {
                int a = stack * (slices + 1) + slice;
                int b = a + slices + 1;
                m.TriangleIndices.Add(a);
                m.TriangleIndices.Add(b + 1);
                m.TriangleIndices.Add(b);
                m.TriangleIndices.Add(a);
                m.TriangleIndices.Add(a + 1);
                m.TriangleIndices.Add(b + 1);
            }
        }
        return m;
    }
    // Create a simple grid mesh in the XZ plane, centered at the origin, with given size and step.
    public static MeshGeometry3D CreateGrid(double size = 10, double step = 1.0)
    {
        var m = new MeshGeometry3D();
        // simple thin lines as flat rectangles
        double half = size / 2.0;
        double thickness = 0.01;
        for (double x = -half; x <= half + 1e-6; x += step)
        {
            var line = CreateBox(thickness, 0.001, size);
            var t = new TranslateTransform3D(x, 0, 0);
            // transform positions and append
            int baseIndex = m.Positions.Count;
            foreach (var p in line.Positions)
                m.Positions.Add(t.Transform(p));
            foreach (var idx in line.TriangleIndices)
                m.TriangleIndices.Add(baseIndex + idx);
        }
        for (double z = -half; z <= half + 1e-6; z += step)
        {
            var line = CreateBox(size, 0.001, thickness);
            var t = new TranslateTransform3D(0, 0, z);
            int baseIndex = m.Positions.Count;
            foreach (var p in line.Positions)
                m.Positions.Add(t.Transform(p));
            foreach (var idx in line.TriangleIndices)
                m.TriangleIndices.Add(baseIndex + idx);
        }
        m.Normals = CalculateNormals(m);
        return m;
    }
    // Create a cylinder mesh along the Y-axis, centered at the origin, with given height and radius.
    public static MeshGeometry3D CreateCylinder(double height, double radius, int segments = 24)
    {
        var m = new MeshGeometry3D();
        double half = height / 2.0;
        // side vertices
        for (int i = 0; i <= segments; i++)
        {
            double t = 2.0 * Math.PI * i / segments;
            double x = Math.Cos(t) * radius;
            double z = Math.Sin(t) * radius;
            m.Positions.Add(new Point3D(x, -half, z)); // bottom ring
            m.Positions.Add(new Point3D(x, half, z));  // top ring
            var nx = Math.Cos(t);
            var nz = Math.Sin(t);
            m.Normals.Add(new Vector3D(nx, 0, nz));
            m.Normals.Add(new Vector3D(nx, 0, nz));
            m.TextureCoordinates.Add(new System.Windows.Point((double)i / segments, 1));
            m.TextureCoordinates.Add(new System.Windows.Point((double)i / segments, 0));
        }

        // side triangles
        for (int i = 0; i < segments; i++)
        {
            int i0 = i * 2;
            int i1 = i0 + 1;
            int i2 = ((i + 1) * 2) % ((segments + 1) * 2);
            int i3 = i2 + 1;
            m.TriangleIndices.Add(i0); m.TriangleIndices.Add(i3); m.TriangleIndices.Add(i1);
            m.TriangleIndices.Add(i0); m.TriangleIndices.Add(i2); m.TriangleIndices.Add(i3);
        }
        // top and bottom caps
        int baseCenterIndex = m.Positions.Count;
        m.Positions.Add(new Point3D(0, -half, 0)); // base center
        m.Normals.Add(new Vector3D(0, -1, 0));
        m.TextureCoordinates.Add(new System.Windows.Point(0.5, 0.5));

        for (int i = 0; i <= segments; i++)
        {
            int p = i * 2; // bottom ring index
            m.TriangleIndices.Add(baseCenterIndex);
            m.TriangleIndices.Add(p);
            m.TriangleIndices.Add(((i + 1) * 2) % ((segments + 1) * 2));
        }
        // top cap
        int topCenterIndex = m.Positions.Count;
        m.Positions.Add(new Point3D(0, half, 0)); // top center
        m.Normals.Add(new Vector3D(0, 1, 0));
        m.TextureCoordinates.Add(new System.Windows.Point(0.5, 0.5));

        for (int i = 0; i <= segments; i++)
        {
            int p = i * 2 + 1; // top ring
            m.TriangleIndices.Add(topCenterIndex);
            m.TriangleIndices.Add(((i + 1) * 2 + 1) % ((segments + 1) * 2));
            m.TriangleIndices.Add(p);
        }

        // ensure normals for side already set; if missing compute
        if (m.Normals.Count != m.Positions.Count)
        {
            m.Normals = CalculateNormals(m);
        }

        return m;
    }
    // Calculate vertex normals for a mesh by averaging face normals.
    private static Vector3DCollection CalculateNormals(MeshGeometry3D mesh)
    {
        var normals = new Vector3DCollection(new Vector3D[mesh.Positions.Count]);
        var counts = new int[mesh.Positions.Count];
        for (int i = 0; i < mesh.TriangleIndices.Count; i += 3)
        {
            int i0 = mesh.TriangleIndices[i];
            int i1 = mesh.TriangleIndices[i + 1];
            int i2 = mesh.TriangleIndices[i + 2];
            var p0 = mesh.Positions[i0];
            var p1 = mesh.Positions[i1];
            var p2 = mesh.Positions[i2];
            var v1 = p1 - p0;
            var v2 = p2 - p0;
            var n = Vector3D.CrossProduct(v1, v2);
            n.Normalize();
            normals[i0] += n; counts[i0]++;
            normals[i1] += n; counts[i1]++;
            normals[i2] += n; counts[i2]++;
        }
        for (int i = 0; i < normals.Count; i++)
        {
            if (counts[i] > 0)
            {
                normals[i] = normals[i] / counts[i];
                normals[i].Normalize();
            }
            else normals[i] = new Vector3D(0, 1, 0);
        }
        return normals;
    }
}
