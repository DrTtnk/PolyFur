using UnityEngine;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter))]
public class KdTreeTest : MonoBehaviour {
    private KdTreeData _data;

    public bool autoInit;

    void Awake() { if (autoInit) Init(); }

    void Init() {
    }

    public bool ClosestPointOnOptimized(Vector3 to, out Vector3 closestPoint, /*ref int triangleCacheIndex,*/ float range = -1f) {
        return _data.ClosestPointOnOptimized(to, out closestPoint, transform, range);
    }

    public bool visualization;

    void OnDrawGizmos() {
        if (!visualization)
            return;
        
        _data = new KdTreeData(GetComponent<MeshFilter>().sharedMesh);
        _data.Build();

        Gizmos.matrix = transform.localToWorldMatrix;
        
        KdTreeData.RecursiveDraw(_data.rootNode, 0, true);
    }
}

