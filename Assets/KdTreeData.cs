using UnityEngine;
using System.Collections.Generic;

public class KdTreeData
{
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

    private int _triangleCount;
    private int _vertexCount;
    private readonly Vector3[] _vertices;

    private readonly TrisLib.Tris[] _triangles;

    private readonly int[] _tris;

    private Vector3[] _triangleCentroid;

    private float[] _triangleRadius;

    private bool[] _needToCheck;

    private int[] _triangleToClear;

    public Node rootNode;

    public KdTreeData(Mesh mesh)
    {
        _tris = mesh.triangles;
        _vertices = mesh.vertices;
    }

    public void Build()
    {
        _vertexCount = _vertices.Length;
        _triangleCount = _tris.Length / 3;

        _triangleCentroid = new Vector3 [_triangleCount];
        _triangleRadius = new float [_triangleCount];

        _needToCheck = new bool [_triangleCount];
        _triangleToClear = new int [_triangleCount];

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

            _triangleRadius[i / 3] =
                Mathf.Max(
                    Vector3.Distance(a, mean),
                    Vector3.Distance(b, mean),
                    Vector3.Distance(c, mean)
                ) + float.Epsilon + 0.00000001f;

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

        for (var i = 0; i < _tris.Length; i += 3)
        {
            rootTriangles.Add(i);
        }

        rootNode = new Node
        {
            partitionAxis = -1,
            bounds = MakeBounds(rootTriangles)
        };


        RecursivePartition(rootTriangles, 0, rootNode, -1);
    }

    private Bounds MakeBounds(List<int> triangles)
    {
        var maxExtents = new Vector3(-float.MaxValue, -float.MaxValue, -float.MaxValue);
        var minExtents = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);

        triangles.ForEach((triangle) =>
        {
            // partitionMeanPoint += centroids[triangle/3];

            minExtents.x = Mathf.Min(minExtents.x, Mathf.Min(_vertices[_tris[triangle]].x, Mathf.Min(_vertices[_tris[triangle + 1]].x, Mathf.Min(_vertices[_tris[triangle + 2]].x))));
            minExtents.y = Mathf.Min(minExtents.y, Mathf.Min(_vertices[_tris[triangle]].y, Mathf.Min(_vertices[_tris[triangle + 1]].y, Mathf.Min(_vertices[_tris[triangle + 2]].y))));
            minExtents.z = Mathf.Min(minExtents.z, Mathf.Min(_vertices[_tris[triangle]].z, Mathf.Min(_vertices[_tris[triangle + 1]].z, Mathf.Min(_vertices[_tris[triangle + 2]].z))));

            maxExtents.x = Mathf.Max(maxExtents.x, Mathf.Max(_vertices[_tris[triangle]].x, Mathf.Max(_vertices[_tris[triangle + 1]].x, Mathf.Max(_vertices[_tris[triangle + 2]].x))));
            maxExtents.y = Mathf.Max(maxExtents.y, Mathf.Max(_vertices[_tris[triangle]].y, Mathf.Max(_vertices[_tris[triangle + 1]].y, Mathf.Max(_vertices[_tris[triangle + 2]].y))));
            maxExtents.z = Mathf.Max(maxExtents.z, Mathf.Max(_vertices[_tris[triangle]].z, Mathf.Max(_vertices[_tris[triangle + 1]].z, Mathf.Max(_vertices[_tris[triangle + 2]].z))));
        });

        return new Bounds { min = minExtents, max = maxExtents };
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

        // center of bounding box
        // Vector3 partitionMeanPoint = parent.bounds.center;
        var parentBounds = parent.bounds;
        var parentBoundsSize = parentBounds.size;

        var partitionAxis = 0;
        var maxSize = parentBoundsSize.x;

        //is Y axis bigger?
        if (maxSize < parentBoundsSize.y)
        {
            partitionAxis = 1;
            maxSize = parentBoundsSize.y;
        }

        //is Z axis biggeR?
        if (maxSize < parentBoundsSize.z)
        {
            partitionAxis = 2;
            maxSize = parentBoundsSize.z;
        }

        //area that doesn't change when re-positioning splitting plane
        var sideArea = 2 * parentBoundsSize[(partitionAxis + 1) % 3] * parentBoundsSize[(partitionAxis + 2) % 3];

        //area that changes when re-positioning splitting plane
        var axisArea = 2 * parentBoundsSize[partitionAxis] * (parentBoundsSize[(partitionAxis + 1) % 3] + parentBoundsSize[(partitionAxis + 2) % 3]);

        var startPos = parentBounds.min[partitionAxis];
        var endPos = parentBounds.max[partitionAxis];

        float minPosition = 0;
        var minHeuristic = System.Single.MaxValue;

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
        var k = 8;
        while (k > 0)
        {
            // desna razlika
            var rightH = Heuristic(triangles, minPosition + step, startPos, endPos, partitionAxis, axisArea,
                sideArea);

            // leva razlika 
            var leftH = Heuristic(triangles, minPosition - step, startPos, endPos, partitionAxis, axisArea,
                sideArea);

            // Count(triangles, partitionCoordinate, partitionAxis, out posCount, out negCount);
            // int trianglesInBoth = triangles.Count - posCount + negCount;

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

            k--;
        }

        partitionCoordinate = minPosition;

        //partitionMeanPoint[partitionAxis] = partitionCoordinate;
        // DebugDraw.DrawMarker(transform.TransformPoint(partitionMeanPoint), 1f, Color.red, 10f, false);

        var positiveTriangles = GetList();
        var negativeTriangles = GetList();

        Split(triangles, partitionCoordinate, partitionAxis, positiveTriangles, negativeTriangles);

        ReturnList(triangles);

        parent.partitionAxis = partitionAxis;
        parent.partitionCoordinate = partitionCoordinate;

        //Vector3 planeNormal = Vector3.zero;

        //planeNormal[partitionAxis] = 1;

        //DebugDraw.DrawPlane(transform.TransformPoint(partitionMeanPoint), planeNormal, extentsMagnitude[partitionAxis]/2f, Color.Lerp(Color.red, Color.blue, (float)depth / 10f), 10 - depth, false);

        // POSITIVE NODE
        var posMin = parent.bounds.min;
        posMin[partitionAxis] = partitionCoordinate;

        var posNode = new Node
        {
            bounds = parentBounds
        };
        posNode.bounds.min = posMin;
        parent.positiveChild = posNode;


        /*
#if DEBUG_KD
        Color c = Color.Lerp(Color.red, Color.blue, (float)depth / 10f);
        c[partitionAxis] = 1f;
        DebugDraw.DrawBounds(parent.positiveChild.bounds, c, 10f - depth, transform);
#endif*/

        // NEGATIVE NODE
        var negMax = parent.bounds.max;
        negMax[partitionAxis] = partitionCoordinate;

        var negNode = new Node
        {
            bounds = parentBounds
        };
        negNode.bounds.max = negMax;
        parent.negativeChild = negNode;

        // DebugDraw.DrawBounds(parent.negativeChild.bounds, c, 10f - depth, transform);

        if (positiveTriangles.Count < triCount && positiveTriangles.Count >= 5)
        {
            RecursivePartition(positiveTriangles, depth + 1, posNode, partitionAxis);
        }
        else
        {
            posNode.triangles = positiveTriangles.ToArray();

            ReturnList(positiveTriangles);

            /*if (drawMeshTreeOnStart)
                DrawTriangleSet(posNode.triangles, DebugDraw.RandomColor(), depth);*/
        }


        if (negativeTriangles.Count < triCount && negativeTriangles.Count >= 5)
        {
            RecursivePartition(negativeTriangles, depth + 1, negNode, partitionAxis);
        }
        else
        {
            negNode.triangles = negativeTriangles.ToArray();

            ReturnList(negativeTriangles);

            /*if (drawMeshTreeOnStart)
                DrawTriangleSet(negNode.triangles, DebugDraw.RandomColor(), depth);*/
        }
    }

    private void Split(List<int> triangles, float partitionCoordinate, int partitionAxis, List<int> positiveTriangles, List<int> negativeTriangles)
    {
        foreach (var triangle in triangles)
        {
            var firstPointAbove = PointAbovePlane(partitionCoordinate, partitionAxis, _vertices[_tris[triangle]]);
            var secondPointAbove = PointAbovePlane(partitionCoordinate, partitionAxis, _vertices[_tris[triangle + 1]]);
            var thirdPointAbove = PointAbovePlane(partitionCoordinate, partitionAxis, _vertices[_tris[triangle + 2]]);

            if (firstPointAbove && secondPointAbove && thirdPointAbove)
            {
                positiveTriangles.Add(triangle);
            }
            else if (!firstPointAbove && !secondPointAbove && !thirdPointAbove)
            {
                negativeTriangles.Add(triangle);
            }
            else
            {
                positiveTriangles.Add(triangle);
                negativeTriangles.Add(triangle);
            }
        }
    }

    //SAH HEURISTICS
    private float Heuristic(List<int> triangles, float partitionCoordinate, float partitionStart, float partitionEnd,
        int partitionAxis, float axisArea, float sideArea)
    {
        var positiveCount = 0;
        var negativeCount = 0;

        for (var i = 0; i < triangles.Count; i++)
        {
            var triangle = triangles[i];

            var firstPointAbove = PointAbovePlane(partitionCoordinate, partitionAxis, _vertices[_tris[triangle + 0]]);
            var secondPointAbove = PointAbovePlane(partitionCoordinate, partitionAxis, _vertices[_tris[triangle + 1]]);
            var thirdPointAbove = PointAbovePlane(partitionCoordinate, partitionAxis, _vertices[_tris[triangle + 2]]);

            if (firstPointAbove && secondPointAbove && thirdPointAbove)
            {
                positiveCount += 1;
            }
            else if (!firstPointAbove && !secondPointAbove && !thirdPointAbove)
            {
                negativeCount += 1;
            }
            else
            {
                //positiveCount += 1;
                //negativeCount += 1;

                //positiveArea += areas[triangle / 3];
                //negativeArea += areas[triangle / 3];
            }
        }

        var ratioLeft = (partitionCoordinate - partitionStart) / (partitionEnd - partitionStart);

        return positiveCount * (sideArea + axisArea * (1f - ratioLeft))
               + negativeCount * (sideArea + axisArea * ratioLeft);

        /*return positiveCount * Mathf.Sqrt((1f - ratioLeft))
             + negativeCount * Mathf.Sqrt((ratioLeft));*/
    }

    private bool PointAbovePlane(Vector3 planeOrigin, Vector3 planeNormal, Vector3 point)
    {
        return Vector3.Dot(point - planeOrigin, planeNormal) >= 0;
    }

    private bool PointAbovePlane(float planeCoordinate, int planeAxis, Vector3 point)
    {
        return point[planeAxis] - planeCoordinate >= 0;
    }

    private float PointDistanceFromPlane(Vector3 planeOrigin, Vector3 planeNormal, Vector3 point)
    {
        return Mathf.Abs(Vector3.Dot(point - planeOrigin, planeNormal));
    }

    // x normal 0 (yz plane)
    // y normal 1 (xz plane)
    // z normal 2 (xy plane)
    private float PointDistanceFromPlane(float planeCoordinate, int planeAxis, Vector3 point)
    {
        return Mathf.Abs(point[planeAxis] - planeCoordinate);
    }

    /*float PointDistanceFromPlaneNoAbs(Vector3 planeOrigin, Vector3 planeNormal, Vector3 point)
    {
        return Vector3.Dot(point - planeOrigin, planeNormal);
    }*/

    private float PointDistanceFromPlaneNoAbs(float planeCoordinate, int planeAxis, Vector3 point)
    {
        return point[planeAxis] - planeCoordinate;
    }

    /// <summary>
    /// Determines the closest point between a point and a triangle.
    /// Borrowed from RPGMesh class of the RPGController package for Unity, by fholm
    /// The code in this method is copyrighted by the SlimDX Group under the MIT license:
    /// 
    /// Copyright (c) 2007-2010 SlimDX Group
    /// 
    /// Permission is hereby granted, free of charge, to any person obtaining a copy
    /// of this software and associated documentation files (the "Software"), to deal
    /// in the Software without restriction, including without limitation the rights
    /// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    /// copies of the Software, and to permit persons to whom the Software is
    /// furnished to do so, subject to the following conditions:
    /// 
    /// The above copyright notice and this permission notice shall be included in
    /// all copies or substantial portions of the Software.
    /// 
    /// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    /// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    /// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    /// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    /// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    /// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    /// THE SOFTWARE.
    /// 
    /// </summary>
    /// <param name="point">The point to test.</param>
    /// <param name="vertex1">The first vertex to test.</param>
    /// <param name="vertex2">The second vertex to test.</param>
    /// <param name="vertex3">The third vertex to test.</param>
    /// <param name="result">When the method completes, contains the closest point between the two objects.</param>
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