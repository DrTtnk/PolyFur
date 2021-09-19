using System.Collections.Generic;
using System.Linq;
using DefaultNamespace;
using UnityEngine;
using static TrisLib;
using Random = UnityEngine.Random;

public class TestMesh : MonoBehaviour
{
    public GameObject strands;
    [SerializeField] private int strandSegments;

    private static Tris[] GetTris(Mesh mesh, Transform transform) {
        var worldPos = mesh.vertices.Select(transform.TransformPoint).ToArray();

        return mesh.triangles
            .Split(3)
            .Select(t => new Tris(worldPos[t[0]], worldPos[t[1]], worldPos[t[2]]))
            .ToArray();
    }

    private static Vector3[] GenerateChildren(Tris[] tris) {
        var partialSum = NormalizedPartialSum(tris);

        return Enumerable
            .Repeat(0, 1000)
            .Select(_ => RandomPoint(tris[PickFromDist(partialSum, Random.value)]))
            .ToArray();
    }

    private static float[] NormalizedPartialSum(Tris[] tris) {
        var total = 0f;
        return tris.Select(t => total += t.W).ToList().Select(x => x / total).ToArray();
    }

    private static int PickFromDist(IReadOnlyList<float> partial, float value) {
        for (var i = 0; i < partial.Count; i++)
            if ((i == 0 || partial[i - 1] < value) && value <= partial[i])
                return i;

        return -1;
    }

    private void OnDrawGizmos()
    {
        var tris = GetTris(GetComponent<MeshFilter>().sharedMesh, transform);

        var points = GenerateChildren(tris);
        
        Gizmos.color = Color.blue;
        var strandVertices = strands
            .GetComponent<MeshFilter>()
            .sharedMesh.vertices
            .Select(strands.transform.TransformPoint)
            .ToList();
        
        strandVertices
            .Split(strandSegments)
            .ForEach(strand => strand.PairUp().ForEach(p => Gizmos.DrawLine(p.first, p.second)));
        
        foreach (var vert in strandVertices)
            Gizmos.DrawSphere(vert, .01f);
        
        // Gizmos.color = Color.red;
        // foreach (var p in points)
        //     Gizmos.DrawSphere(p, .01f);
    }
}