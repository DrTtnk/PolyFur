using System.Linq;
using DefaultNamespace;
using UnityEngine;
using Random = UnityEngine.Random;
// ReSharper disable InconsistentNaming

public class TrisLib
{
    public class Tris
    {
        public Vector3 p1;
        public Vector3 p2;
        public Vector3 p3;
        public readonly float W;
        public readonly Vector3 N;

        public Tris(Vector3 p1, Vector3 p2, Vector3 p3) {
            this.p1 = p1;
            this.p2 = p2;
            this.p3 = p3;
            W = Vector3.Cross(p2 - p1, p3 - p1).magnitude * .5f;
            N = Vector3.Cross(p2 - p1, p3 - p1).normalized;
        }
    }

    public static Tris[] GetTris(Mesh mesh) => mesh.triangles
        .Split(3)
        .Select(t => new Tris(mesh.vertices[t[0]], mesh.vertices[t[1]], mesh.vertices[t[2]]))
        .ToArray();

    public static Vector3 RandomPoint(Tris tris) {
        var a = tris.p2 - tris.p1;
        var b = tris.p3 - tris.p1;

        var (u1, u2) = (Random.value, Random.value);

        var w = u1 + u2 > 1 
            ? (1 - u1) * a + (1 - u2) * b 
            : u1 * a + u2 * b;

        return w + tris.p1;
    }
    
    public static float Area(Vector3 A, Vector3 B, Vector3 C) => Vector3.Cross(B - A, C - A).magnitude * .5f;

    public static Vector3 Barycentric(Vector3 p, Tris t) {
        var a = Area(t.p3, t.p2, p) / t.W;
        var b = Area(t.p1, t.p3, p) / t.W;

        return new Vector3(a, b, 1f - a - b);
    }
    
    public static bool IsInTheTriangle(Vector3 p, Tris t, out Vector3 barycentric) {
        if(Vector3.Dot(t.N, p - t.p1) < Mathf.Epsilon) {
            barycentric = Vector3.zero;
            return false;
        }

        barycentric = Barycentric(p, t);

        return Mathf.Abs(barycentric.x + barycentric.y + barycentric.z - 1f) < Mathf.Epsilon;
    }
}
