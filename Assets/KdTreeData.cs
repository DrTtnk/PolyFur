using UnityEngine;
using System.Collections.Generic;
using System.Linq;

public class KdTreeData
{
    public class BBox {
        public Vector3 min;
        public Vector3 max;
        public Vector3 Center => .5f * (max + min);

        public BBox() { }

        public BBox(Vector3 p1, Vector3 p2, Vector3 p3) {
            min = Vector3.Min(p1, Vector3.Min(p2, p3));
            max = Vector3.Max(p1, Vector3.Max(p2, p3));
        }
    }
    
    public class BvhTris: TrisLib.Tris
    {
        public readonly Vector3 centroid;

        public float radius;

        public bool toClear = true;

        public bool toCheck;

        public BvhTris(TrisLib.Tris tris): base(tris.p1, tris.p2, tris.p3) {
            centroid = (p1 + p2 + p3) / 3f;

            radius = Mathf.Max(
                Vector3.Distance(p1, centroid), 
                Vector3.Distance(p2, centroid), 
                Vector3.Distance(p3, centroid)
            ) + float.Epsilon + 0.00000001f;
        }
    }
    
    public class Node
    {
        public float partitionCoordinate;
        public int partitionAxis;

        public Node positiveChild;
        public Node negativeChild;

        public int[] triangles;

        public Bounds bounds;

        public Vector3 tempClosestPoint;
    };

    private readonly Vector3[] _vertices;
    private readonly int[] _tris;

    private readonly BvhTris[] _triangles;


    private Vector3[] _triangleCentroid;

    private float[] _triangleRadius;

    private bool[] _needToCheck;

    private int[] _triangleToClear;

    public Node rootNode;

    public KdTreeData(Mesh mesh)
    {
        _triangles = TrisLib.GetTris(mesh).Select(t => new BvhTris(t)).ToArray();
        _tris = mesh.triangles;
        _vertices = mesh.vertices;
    }

    public void Build()
    {
        _triangleCentroid = new Vector3 [_triangles.Length];
        _triangleRadius = new float [_triangles.Length];

        _needToCheck = new bool [_triangles.Length];
        _triangleToClear = new int [_triangles.Length];

        for (var i = 0; i < _tris.Length; i += 3)
        {
            var indexA = _tris[i + 0];
            var indexB = _tris[i + 1];
            var indexC = _tris[i + 2];

            var a = _vertices[indexA];
            var b = _vertices[indexB];
            var c = _vertices[indexC];

            var mean = (a + b + c) / 3f;
            _triangleCentroid[i / 3] = mean;

            _triangleRadius[i / 3] = Mathf.Max(Vector3.Distance(a, mean), Vector3.Distance(b, mean), Vector3.Distance(c, mean)) + float.Epsilon + 0.00000001f;

            _needToCheck[i / 3] = true;
        }

        BuildTriangleTree();

        _listPool.Clear();
    }

    private readonly Stack<Node> _nodesToProcess = new Stack<Node>(1000);

    public bool ClosestPointOnOptimized(Vector3 to, out Vector3 closestPoint, Transform transform, float range = -1f) {
        to = transform.InverseTransformPoint(to);

        _nodesToProcess.Clear();

        float currentMin = 0;
        var currClosestTriangle = 0;
        var triangleToClearCount = 0;
        if (range != -1f)
        {
            var scl = transform.lossyScale;
            range /= Mathf.Min(scl.x, Mathf.Min(scl.y, scl.z));

            currentMin = range;
        }
        else
        {
            currentMin = 9999;
        }

        rootNode.tempClosestPoint = rootNode.bounds.ClosestPoint(to);

        // push root tree
        _nodesToProcess.Push(rootNode);

        var checkCount = 0;
        while (_nodesToProcess.Count > 0)
        {
            var node = _nodesToProcess.Pop();

            if (node.triangles == null)
            {
                var partitionAxis = node.partitionAxis;
                var partitionCoord = node.partitionCoordinate;

                var tempClosestPoint = node.tempClosestPoint;

                // Calculating closest distance to bounding box

                //inside positive side, project on negative
                if (tempClosestPoint[partitionAxis] - partitionCoord >= 0)
                {
                    node.positiveChild.tempClosestPoint = tempClosestPoint;

                    tempClosestPoint[partitionAxis] = partitionCoord;
                    node.negativeChild.tempClosestPoint = tempClosestPoint;

                    //we are inside positive bound, we don't need to test for distance
                    _nodesToProcess.Push(node.positiveChild);

                    if (Vector3.SqrMagnitude(node.negativeChild.tempClosestPoint - to) <= currentMin * currentMin && node.negativeChild.triangles != null && node.negativeChild.triangles.Length != 0)
                        _nodesToProcess.Push(node.negativeChild);
                }
                else //inside negative side, project on positive
                {
                    node.negativeChild.tempClosestPoint = tempClosestPoint;

                    tempClosestPoint[partitionAxis] = partitionCoord;
                    node.positiveChild.tempClosestPoint = tempClosestPoint;

                    if (Vector3.SqrMagnitude(node.positiveChild.tempClosestPoint - to) <= currentMin * currentMin && node.positiveChild.triangles != null && node.positiveChild.triangles.Length != 0)

                        _nodesToProcess.Push(node.positiveChild);

                    //we are inside negative bound, we don't need to test for distance
                    _nodesToProcess.Push(node.negativeChild);
                }
            }
            else
            {
                // preglejmo vse trikotnike, vzemimo najbližjega
                foreach (var t in node.triangles)
                {
                    checkCount++;

                    var newClosestTriangle = t;

                    var dividedIndex = newClosestTriangle / 3;

                    if (_needToCheck[dividedIndex])
                    {
                        _needToCheck[dividedIndex] = false;
                        _triangleToClear[triangleToClearCount] = dividedIndex;
                        triangleToClearCount++;
                    }
                    else
                        continue; //already checked, move along

                    // approx test
                    if (MinDistanceToTriangleApprox(newClosestTriangle, to) <= currentMin)
                    {
                        // actual test
                        var newMin = ClosestDistanceOnTriangleSingle(newClosestTriangle, to);
#if DEBUG_KD
                        DebugDraw.DrawTriangle(
                            transform.TransformPoint(vertices[tris[newClosestTriangle    ]]),
                            transform.TransformPoint(vertices[tris[newClosestTriangle + 1]]),
                            transform.TransformPoint(vertices[tris[newClosestTriangle + 2]]),
                            Color.yellow
                        );
#endif
                        // je ta trikotnik bližje?
                        if (newMin <= currentMin)
                        {
                            currentMin = newMin;
                            currClosestTriangle = newClosestTriangle;
                        }
                    }
                }
            }
        }

#if DEBUG_KD
        Debug.Log("checkCount:" + checkCount);
        Debug.Log("actualTrianglesTested:" + triangleToClearCount);
#endif
        while (triangleToClearCount > 0)
        {
            triangleToClearCount--;
            _needToCheck[_triangleToClear[triangleToClearCount]] = true;
        }

        if (checkCount == 0)
        {
            closestPoint = Vector3.zero;
            return false;
        }

        /*foreach (bool b in needToCheck)
        {
            if (!b)
                Debug.Log("GOTYOU");
        }
        */
        var closest = Vector3.zero;

        ClosestPointOnTriangleToPoint(
            ref _vertices[_tris[currClosestTriangle]],
            ref _vertices[_tris[currClosestTriangle + 1]],
            ref _vertices[_tris[currClosestTriangle + 2]],
            ref to,
            out closest);

#if DEBUG_KD
        DebugDraw.DrawTriangle(
            transform.TransformPoint(vertices[tris[currClosestTriangle    ]]), 
            transform.TransformPoint(vertices[tris[currClosestTriangle + 1]]), 
            transform.TransformPoint(vertices[tris[currClosestTriangle + 2]]),
            Color.red
            );
#endif

        closestPoint = transform.TransformPoint(closest);


        return true;
    }

    // temp
    private Vector3 _p1;
    private Vector3 _p2;
    private Vector3 _p3;
    private Vector3 _nearest;

    private float ClosestDistanceOnTriangleSingle(int triangle, Vector3 to)
    {
        _p1 = _vertices[_tris[triangle]];
        _p2 = _vertices[_tris[triangle + 1]];
        _p3 = _vertices[_tris[triangle + 2]];

        ClosestPointOnTriangleToPoint(ref _p1, ref _p2, ref _p3, ref to, out _nearest);

        return Vector3.Magnitude(to - _nearest);
    }

    /// <summary>
    /// Can be negative in weird cases, still you need to check
    /// </summary>
    /// <param name="triangle"></param>
    /// <param name="to"></param>
    /// <returns></returns>
    private float MinDistanceToTriangleApprox(int triangle, Vector3 to)
    {
        var triangleIndex = triangle / 3;

        return Vector3.Distance(_triangleCentroid[triangleIndex], to) - _triangleRadius[triangleIndex];
    }

    private Vector3 ClosestPointOnTriangle(int[] triangles, Vector3 to)
    {
        var shortestDistance = float.MaxValue;

        var shortestPoint = Vector3.zero;

        // Iterate through all triangles
        foreach (var triangle in triangles)
        {
            var p1 = _vertices[_tris[triangle + 0]];
            var p2 = _vertices[_tris[triangle + 1]];
            var p3 = _vertices[_tris[triangle + 2]];

            Vector3 nearest;

            ClosestPointOnTriangleToPoint(ref p1, ref p2, ref p3, ref to, out nearest);

            var distance = (to - nearest).sqrMagnitude;

            if (distance <= shortestDistance)
            {
                shortestDistance = distance;
                shortestPoint = nearest;
            }
        }

        return shortestPoint;
    }

    private void BuildTriangleTree()
    {
        var rootTriangles = new List<int>();

        for (var i = 0; i < _tris.Length; i += 3) rootTriangles.Add(i);

        rootNode = new Node
        {
            partitionAxis = -1,
            bounds = MakeBounds(_triangles)
        };


        RecursivePartition(rootTriangles, 0, rootNode, -1);
    }

    private static Bounds MakeBounds(IEnumerable<TrisLib.Tris> tris)
    {
        var boxes = tris.Select(t => new BBox(t.p1, t.p2, t.p3)).ToArray();
        return new Bounds {
            min = boxes.Select(b => b.min).Aggregate(Vector3.Min),
            max = boxes.Select(b => b.max).Aggregate(Vector3.Max)
        };
    }

    private readonly Stack<List<int>> _listPool = new Stack<List<int>>();

    private List<int> GetList()
    {
        if (_listPool.Count <= 0) 
            return new List<int>();
        var l = _listPool.Pop();
        l.Clear();
        return l;
    }

    private void ReturnList(List<int> list) => _listPool.Push(list);

    private void RecursivePartition(List<int> triangles, int depth, Node parent, int previousAxis)
    {
        var triCount = triangles.Count;

        var partitionCoordinate = 0f;

        var parentBounds = parent.bounds;
        var parentBoundsSize = parentBounds.size;

        var partitionAxis = 0;
        var maxSize = parentBoundsSize.x;

        //is Y axis bigger?
        if (maxSize < parentBoundsSize.y) { partitionAxis = 1; maxSize = parentBoundsSize.y; }
        
        //is Z axis bigger?
        if (maxSize < parentBoundsSize.z) { partitionAxis = 2; maxSize = parentBoundsSize.z; }

        //area that doesn't change when re-positioning splitting plane
        var sideArea = 2 * parentBoundsSize[(partitionAxis + 1) % 3] * parentBoundsSize[(partitionAxis + 2) % 3];

        //area that changes when re-positioning splitting plane
        var axisArea = 2 * parentBoundsSize[partitionAxis] * (parentBoundsSize[(partitionAxis + 1) % 3] + parentBoundsSize[(partitionAxis + 2) % 3]);

        var startPos = parentBounds.min[partitionAxis];
        var endPos = parentBounds.max[partitionAxis];

        float minPosition = 0;
        var minHeuristic = float.MaxValue;

        var areas = Mathf.Max(triangles.Count / 20, 5);

        var step = (endPos - startPos) / areas;

        // sweep test
        for (var i = 1; i < areas; i++)
        {
            var pos = startPos + step * i;

            var h = Heuristic(triangles, pos, startPos, endPos, partitionAxis, axisArea, sideArea);

            if (h < minHeuristic)
            {
                minHeuristic = h;
                minPosition = pos;
            }
        }

        step /= 2f;

        // making partition, bisection
        // maximum 12 iterations!
        // works OK
        for (var k = 0; k <10; k++)
        {
            var rightH = Heuristic(triangles, minPosition + step, startPos, endPos, partitionAxis, axisArea, sideArea);
            var leftH  = Heuristic(triangles, minPosition - step, startPos, endPos, partitionAxis, axisArea, sideArea);

            //moving right is positive, it makes heuristic smaller
            if (rightH < minHeuristic && rightH < leftH)
            {
                minPosition += step;
                minHeuristic = rightH;
            }
            //moving left is negative, it makes heuristic smaller
            else if (leftH < minHeuristic && leftH < rightH)
            {
                minPosition -= step;
                minHeuristic = leftH;
            }
            else
                break;

            //minimize step by two
            step /= 2f;
        }

        partitionCoordinate = minPosition;

        var positiveTriangles = GetList();
        var negativeTriangles = GetList();

        var (positives, negatives) = Split(triangles, partitionCoordinate, partitionAxis);
        positiveTriangles.AddRange(positives);
        negativeTriangles.AddRange(negatives);
        
        ReturnList(triangles);

        parent.partitionAxis = partitionAxis;
        parent.partitionCoordinate = partitionCoordinate;

        // POSITIVE NODE
        var posMin = parent.bounds.min;
        posMin[partitionAxis] = partitionCoordinate;

        var posNode = new Node { bounds = parentBounds };
        posNode.bounds.min = posMin;
        parent.positiveChild = posNode;

        // NEGATIVE NODE
        var negMax = parent.bounds.max;
        negMax[partitionAxis] = partitionCoordinate;

        var negNode = new Node { bounds = parentBounds };
        negNode.bounds.max = negMax;
        parent.negativeChild = negNode;

        if (positiveTriangles.Count < triCount && positiveTriangles.Count >= 5)
        {
            RecursivePartition(positiveTriangles, depth + 1, posNode, partitionAxis);
        }
        else
        {
            posNode.triangles = positiveTriangles.ToArray();

            ReturnList(positiveTriangles);
        }

        if (negativeTriangles.Count < triCount && negativeTriangles.Count >= 5)
        {
            RecursivePartition(negativeTriangles, depth + 1, negNode, partitionAxis);
        }
        else
        {
            negNode.triangles = negativeTriangles.ToArray();
            ReturnList(negativeTriangles);
        }
    }

    private (List<int> positives, List<int> negatives) Split(List<int> triangles, float partitionCoordinate, int partitionAxis) {
        bool IsAbove(Vector3 point) => point[partitionAxis] - partitionCoordinate >= 0;

        var positions = triangles.Select(t => (
            above1: IsAbove(_vertices[_tris[t + 0]]),
            above2: IsAbove(_vertices[_tris[t + 1]]),
            above3: IsAbove(_vertices[_tris[t + 2]]),
            tris: t
        )).ToArray();
        
        var neither = positions.Where(p => !(p.above1 && p.above2 && p.above3 || !p.above1 && !p.above2 && !p.above3)).ToList();
        var positives = positions.Where(p => p.above1 && p.above2 && p.above3).ToList();
        var negatives = positions.Where(p => !p.above1 && !p.above2 && !p.above3).ToList();

        return (
            positives: positives.Concat(neither).Select(p => p.tris).ToList(),
            negatives: negatives.Concat(neither).Select(p => p.tris).ToList()
        );
    }

    //SAH HEURISTICS
    private float Heuristic(List<int> triangles, float partitionCoordinate, float partitionStart, float partitionEnd, int partitionAxis, float axisArea, float sideArea) {
        bool IsAbove(Vector3 point) => point[partitionAxis] - partitionCoordinate >= 0;

        var positions = triangles.Select(t => (
            firstAbove: IsAbove(_vertices[_tris[t + 0]]),
            secondAbove: IsAbove(_vertices[_tris[t + 1]]),
            thirdAbove: IsAbove(_vertices[_tris[t + 2]])
        )).ToArray();
        
        var positiveCount = positions.Count(p => p.firstAbove && p.secondAbove && p.thirdAbove);
        var negativeCount = positions.Count(p => !p.firstAbove && !p.secondAbove && !p.thirdAbove);

        var ratioLeft = (partitionCoordinate - partitionStart) / (partitionEnd - partitionStart);

        return positiveCount * (sideArea + axisArea * (1f - ratioLeft)) + negativeCount * (sideArea + axisArea * ratioLeft);
    }

    // Determines the closest point between a point and a triangle.
    private void ClosestPointOnTriangleToPoint(ref Vector3 vertex1, ref Vector3 vertex2, ref Vector3 vertex3, ref Vector3 point, out Vector3 result)
    {
        //Source: Real-Time Collision Detection by Christer Ericson
        //Reference: Page 136

        //Check if P in vertex region outside A
        var ab = vertex2 - vertex1;
        var ac = vertex3 - vertex1;
        var ap = point - vertex1;

        var d1 = Vector3.Dot(ab, ap);
        var d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            result = vertex1; //Barycentric coordinates (1,0,0)
            return;
        }

        //Check if P in vertex region outside B
        var bp = point - vertex2;
        var d3 = Vector3.Dot(ab, bp);
        var d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3)
        {
            result = vertex2; // barycentric coordinates (0,1,0)
            return;
        }

        //Check if P in edge region of AB, if so return projection of P onto AB
        var vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            var v = d1 / (d1 - d3);
            result = vertex1 + v * ab; //Barycentric coordinates (1-v,v,0)
            return;
        }

        //Check if P in vertex region outside C
        var cp = point - vertex3;
        var d5 = Vector3.Dot(ab, cp);
        var d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6)
        {
            result = vertex3; //Barycentric coordinates (0,0,1)
            return;
        }

        //Check if P in edge region of AC, if so return projection of P onto AC
        var vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            var w = d2 / (d2 - d6);
            result = vertex1 + w * ac; //Barycentric coordinates (1-w,0,w)
            return;
        }

        //Check if P in edge region of BC, if so return projection of P onto BC
        var va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && d4 - d3 >= 0.0f && d5 - d6 >= 0.0f)
        {
            var w = (d4 - d3) / (d4 - d3 + (d5 - d6));
            result = vertex2 + w * (vertex3 - vertex2); //Barycentric coordinates (0,1-w,w)
            return;
        }

        //P inside face region. Compute Q through its barycentric coordinates (u,v,w)
        var denom = 1.0f / (va + vb + vc);
        var v2 = vb * denom;
        var w2 = vc * denom;
        result = vertex1 + ab * v2 + ac * w2; //= u*vertex1 + v*vertex2 + w*vertex3, u = va * denom = 1.0f - v - w
    }

    public static void RecursiveDraw(Node node, int depth, bool permutation)
    {
        var color = Color.HSVToRGB(((permutation ? 0.05f : 0f) + depth * 0.05f) % 1f, 1f, 1f);
        color.a = Mathf.Clamp(depth / 10f, 0f, 1f) * 0.1f;
        Gizmos.color = color;

        Gizmos.DrawWireCube(node.bounds.center, node.bounds.size);

        if (node.positiveChild != null) RecursiveDraw(node.positiveChild, depth + 1, permutation);
        if (node.negativeChild != null) RecursiveDraw(node.negativeChild, depth + 1, !permutation);
    }
}