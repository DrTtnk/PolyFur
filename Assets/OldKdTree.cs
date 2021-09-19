using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public struct KdPoint {
    public Vector3 pos;
    public int id;

    public KdPoint(Vector3 pos, int id) {
        this.pos = pos;
        this.id = id;
    }
}

public class KD_Tree {
    private readonly KD_Node _root;

    public KD_Tree(IReadOnlyList<Vector3> points, IReadOnlyList<int> ids) {
        var kdPoints = new KdPoint[points.Count];

        for (var i = 0; i < points.Count; i++)
            kdPoints[i] = new KdPoint(points[i], ids[i]);

        _root = new KD_Node(kdPoints);
    }

    public KdPoint Closest(Vector3 origin) => K_Closest(origin, 1)[0];

    public KdPoint[] K_Closest(Vector3 origin, int k) {
        var minDist = 0f;

        var pos = new KdPoint[k];
        for (var i = 0; i < k; i++) {
            pos[i] = _root.Closest(origin, Mathf.Infinity, minDist, pos[i]);
            minDist = (origin - pos[i].pos).magnitude;
        }
        return pos;
    }
}

public class KD_Node {
    readonly KD_Node childLow;
    readonly KD_Node childHigh;
    Vector3Int axis;
    readonly float separationPlane;

    readonly KdPoint position;
    readonly int idx;

    public KD_Node(KdPoint[] points) {
        if (points.Length == 1) {
            childHigh = childLow = null;
            position = points[0];
            return;
        }

        var mmmX = MinMaxMedian(points.Select(p => p.pos.x).ToArray());
        var mmmY = MinMaxMedian(points.Select(p => p.pos.y).ToArray());
        var mmmZ = MinMaxMedian(points.Select(p => p.pos.z).ToArray());
        
        axis =  new Vector3Int {
            x = mmmX.size > mmmY.size && mmmX.size > mmmZ.size ? 1 : 0,
            y = mmmY.size > mmmX.size && mmmY.size > mmmZ.size ? 1 : 0,
            z = mmmZ.size > mmmX.size && mmmZ.size > mmmY.size ? 1 : 0,
        };;

        if (axis.x == 1)
            Array.Sort(points, (a, b) => a.pos.x.CompareTo(b.pos.x));
        if (axis.y == 1)
            Array.Sort(points, (a, b) => a.pos.y.CompareTo(b.pos.y));
        if (axis.z == 1)
            Array.Sort(points, (a, b) => a.pos.z.CompareTo(b.pos.z));

        var pointsLow = points.Take(points.Length / 2).ToArray();
        var pointsHigh = points.Skip(points.Length / 2).ToArray();

        if (axis.x == 1)
            separationPlane = (pointsLow[pointsLow.Length - 1].pos.x + pointsHigh[0].pos.x) * .5f;
        if (axis.y == 1)
            separationPlane = (pointsLow[pointsLow.Length - 1].pos.y + pointsHigh[0].pos.y) * .5f;
        if (axis.z == 1)
            separationPlane = (pointsLow[pointsLow.Length - 1].pos.z + pointsHigh[0].pos.z) * .5f;

        childLow = new KD_Node(pointsLow);
        childHigh = new KD_Node(pointsHigh);
    }

    private static (float median, float min, float max, float size) MinMaxMedian(IReadOnlyCollection<float> points) => (
        median: points.OrderBy(x => x).Skip(points.Count / 2).First(), 
        min: points.Min(), 
        max: points.Max(),
        size: points.Max() - points.Min()
    );

    public KdPoint Closest(Vector3 origin, float distance, float minDist, KdPoint pos) {
        if (childLow == null) {
            var currDist = (position.pos - origin).magnitude;
            if (currDist < distance && currDist > minDist) 
                return position;
            return pos;
        }

        var originPos = Vector3.Dot(axis, origin);

        if (originPos < separationPlane) {
            if (originPos - distance <= separationPlane)
                return childLow.Closest(origin, distance, minDist, pos);
            if (originPos + distance > separationPlane)
                return childHigh.Closest(origin, distance, minDist, pos);
        } else {
            if (originPos + distance > separationPlane)
                return childHigh.Closest(origin, distance, minDist, pos);
            if (originPos - distance <= separationPlane)
                return childLow.Closest(origin, distance, minDist, pos);
        }
        
        return pos;
    }
}