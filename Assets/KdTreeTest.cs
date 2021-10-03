using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter))]
public class KdTreeTest : MonoBehaviour {
    private KdTreeData _data;

    public bool autoInit;

    public Transform point;

    void Awake() { if (autoInit) Init(); }

    void Init() {
    }

    public Vector3 ClosestPointOnOptimized(Vector3 to, float range = -1f) {
        _data.ClosestPointOnOptimized(to, out Vector3 closestPoint, transform, range);
        return closestPoint;
    }

    public bool visualization;

    void OnDrawGizmos() {
            _data = new KdTreeData(GetComponent<MeshFilter>().sharedMesh);
            _data.Build();

        var position = point.position;
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(ClosestPointOnOptimized(position), .05f);
        Gizmos.DrawSphere(position, .05f);

        Gizmos.matrix = transform.localToWorldMatrix;
        
        if (visualization)
            KdTreeData.RecursiveDraw(_data.rootNode, 0, true);

    }
}

