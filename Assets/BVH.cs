using System.Collections.Generic;
using System.Linq;
using UnityEngine;

class BBox {
    public Vector3 min;
    public Vector3 max;
    public Vector3 Center => .5f * (max + min);

    public BBox() { }

    public BBox(Vector3 p1, Vector3 p2, Vector3 p3) {
        min = Vector3.Min(p1, Vector3.Min(p2, p3));
        max = Vector3.Max(p1, Vector3.Max(p2, p3));
    }
}

class Bvh
{
    private readonly TrisLib.Tris[] _tris;
    private readonly int _maxTrisPerNode;
    private readonly Vector3[] _points;
    private readonly BBox[] _bboxArray;
    public BvhNode root;

    public Bvh(TrisLib.Tris[] tris, int maxTrisPerNode) {
        _tris = tris;
        _maxTrisPerNode = maxTrisPerNode;
        
        _bboxArray = CalcBoundingBoxes(tris);
        root = new BvhNode(0, CalcExtends(_bboxArray));
    }

    private static BBox[] CalcBoundingBoxes(IEnumerable<TrisLib.Tris> tris) => tris
        .Select(t => new BBox(t.p1, t.p2, t.p3))
        .ToArray();
    
    private static BBox CalcExtends(BBox[] boxes) => new BBox{
        min = boxes.Select(b => b.min).Aggregate(Vector3.Min),
        max = boxes.Select(b => b.min).Aggregate(Vector3.Max)
    };
}

class BvhNode
{
    public int level;
    public BBox box;

    public BvhNode(int level, BBox box) {
        this.level = level;
        this.box = box;
    }
}