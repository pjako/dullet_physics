part of dullet_physics;

//
// WebGL Physics Convex Hull helpers.
// (Mostly mirrored with turbulenz/tools/mesh.py)
//
class WebGLPhysicsConvexHullHelpers {
    static bool isPlanar(points) {
        // tolerance for distance from plane for a point
        // to be treat as coplanar.
        var tolerance = WebGLPhysicsConfig.COPLANAR_THRESHOLD;

        var p00 = points[0];
        var p01 = points[1];
        var p02 = points[2];

        // Find normal of plane from first 3 vertices.
        var e10 = (points[3] - p00);
        var e11 = (points[4] - p01);
        var e12 = (points[5] - p02);

        var e20 = (points[6] - p00);
        var e21 = (points[7] - p01);
        var e22 = (points[8] - p02);

        var n0 = (e11 * e22) - (e12 * e21);
        var n1 = (e12 * e20) - (e10 * e22);
        var n2 = (e10 * e21) - (e11 * e20);

        // Though normalisation isn't required to determine if point is 'on' the plane
        // We allow a distance tolerance so normalisation should be performed.
        var normalScale = 1 / Math.sqrt((n0 * n0) + (n1 * n1) + (n2 * n2));
        n0 *= normalScale;
        n1 *= normalScale;
        n2 *= normalScale;

        var planeDistance = -((p00 * n0) + (p01 * n1) + (p02 * n2));

        var i;
        var maxN = points.length;
        for (i = 0; i < maxN; i += 3)
        {
            var distance = (points[i] * n0) + (points[i + 1] * n1) + (points[i + 2] * n2) + planeDistance;
            if ((distance * distance) > tolerance)
            {
                return false;
            }
        }

        return true;
    }

    static WebGLPhysicsPrivateTriangleArray makePlanarConvexHull(dynamic points) {
        var DONT_NORMALIZE_THRESHOLD = 1e-6;

        // Use a 2D graham scan with projections of points onto their maximal plane.
        // Time complexity O(nh) for n points and h out-points.

        // Determine maximal plane for projection as the plane containing points.
        var p00 = points[0];
        var p01 = points[1];
        var p02 = points[2];

        var e10 = (points[3] - p00);
        var e11 = (points[4] - p01);
        var e12 = (points[5] - p02);

        var e20 = (points[6] - p00);
        var e21 = (points[7] - p01);
        var e22 = (points[8] - p02);

        // We do not require normalisation for projection onto plane.
        var normal0 = (e11 * e22) - (e12 * e21);
        var normal1 = (e12 * e20) - (e10 * e22);
        var normal2 = (e10 * e21) - (e11 * e20);

        // Determine tangent vectors.
        var tangent0, tangent1, tangent2;
        if ((normal0 * normal0) + (normal2 * normal2) < DONT_NORMALIZE_THRESHOLD)
        {
            tangent0 = 1;
            tangent1 = tangent2 = 0;
        }
        else
        {
            tangent0 = -normal2;
            tangent1 = 0;
            tangent2 = normal0;
        }
        var bitangent0 = (normal1 * tangent2) - (normal2 * tangent1);
        var bitangent1 = (normal2 * tangent0) - (normal0 * tangent2);
        var bitangent2 = (normal0 * tangent1) - (normal1 * tangent0);

        // Project points.
        var numPoints = points.length / 3;
        var projs = new Float32List(numPoints * 2);
        var p0, p1, p2;
        var i;
        for (i = 0; i < numPoints; i += 1)
        {
            p0 = points[i * 3];
            p1 = points[(i * 3) + 1];
            p2 = points[(i * 3) + 2];

            projs[i * 2] = (p0 * tangent0) + (p1 * tangent1) + (p2 * tangent2);
            projs[(i * 2) + 1] = (p0 * bitangent0) + (p1 * bitangent1) + (p2 * bitangent2);
        }

        // Find first vertex on projected hull as minimal lexicographically.
        var i0 = 0;
        p00 = projs[0];
        p01 = projs[1];
        for (i = 2; i < (numPoints * 2); i += 2)
        {
            p0 = projs[i];
            p1 = projs[i + 1];
            if (p0 < p00 || (p0 == p00 && p1 < p01))
            {
                i0 = (i / 2);
                p00 = p0;
                p01 = p1;
            }
        }

        // Perform graham scan.

        // hullVertices is a mapping for vertices used by hull from
        // their present indices to new indices in output mesh.

        var hullVertices = {};
        hullVertices[i0] = 0;
        var outVertexCount = 1;

        var hullTriangles = [];

        var fsti = i0;
        for (;;)
        {
            var max0, max1, maxDistance;
            var i1 = -1;

            for (i = 0; i < (numPoints * 2); i += 2)
            {
                if (i == (i0 * 2))
                {
                    continue;
                }

                p0 = projs[i];
                p1 = projs[i + 1];
                var plsq = (((p0 - p00) * (p0 - p00)) + ((p1 - p01) * (p1 - p01)));
                if (i1 == -1)
                {
                    i1 = (i / 2);
                    max0 = p0;
                    max1 = p1;
                    maxDistance = plsq;
                    continue;
                }

                // If this is not first vertex tested, determine if new vertex
                // makes a right turn looking in direction of edge, or is further
                // in same direction.
                var turn = ((max0 - p00) * (p1 - p01)) - ((max1 - p01) * (p0 - p00));
                if (turn < 0 || (turn == 0 && plsq > maxDistance))
                {
                    i1 = (i / 2);
                    max0 = p0;
                    max1 = p1;
                    maxDistance = plsq;
                }
            }

            // BUG in TypeScript:
            //   http://typescript.codeplex.com/workitem/145
            if (hullVertices.containsKey(i1)) {
                break;
            }

            // Append vertex i1 to hull
            hullVertices[i1] = outVertexCount;
            outVertexCount += 1;

            // Form triangle (fsti, i0, i1)
            if (i0 != fsti) {
              hullTriangles.add(fsti);
              hullTriangles.add(i0);
              hullTriangles.add(i1);
            }

            i0 = i1;
            p00 = projs[i1 * 2];
            p01 = projs[(i1 * 2) + 1];
        }

        // Output triangle array!
        return createArray(points, hullTriangles, hullVertices, outVertexCount);
    }

    static WebGLPhysicsPrivateTriangleArray makeConvexHull(List<double> points) {
        // 3D generalisation of Graham Scan to facilitate triangulation of the hull in generation
        // Time complexity O(nh) for n points, and h out-points.

        // Find first vertex on hull as minimal lexicographically ordered point.
        var i0 = 0;
        var p00 = points[0];
        var p01 = points[1];
        var p02 = points[2];

        var i;
        var p0, p1, p2;
        var numPoints = (points.length / 3);
        for (i = 3; i < (numPoints * 3); i += 3)
        {
            p0 = points[i];
            p1 = points[i + 1];
            p2 = points[i + 2];
            if (p0 < p00 || (p0 == p00 && (p1 < p01 || (p1 == p01 && p2 < p02))))
            {
                i0 = (i / 3);
                p00 = p0;
                p01 = p1;
                p02 = p2;
            }
        }

        // Find second vertex on hull by performing 2D graham scan step on xy-plane projections of positions
        var i1 = -1;
        var cos1 = -2; // Will always be overriden as cos(theta) > -2
        var lsq1 = 0;
        var d0, d1;
        for (i = 0; i < (numPoints * 3); i += 3)
        {
            if (i == (i0 * 3))
            {
                continue;
            }

            p0 = points[i];
            p1 = points[i + 1];
            d0 = p0 - p00;
            d1 = p1 - p01;
            var lsq = ((d0 * d0) + (d1 * d1));
            if (lsq == 0) {
                if (i1 == -1) {
                    i1 = (i / 3);
                }
                continue;
            }

            var cos = d1 / Math.sqrt(lsq);
            if (cos > cos1 || (cos == cos1 && lsq > lsq1))
            {
                cos1 = cos;
                lsq1 = lsq;
                i1 = (i / 3);
            }
        }

        // Dictionary of visited edges to avoid duplicates
        // List of open edges to be visited by graham scan.
        var closedSet = {};
        var openSet = [i0, i1, i1, i0];

        // Dictionary of vertices used by hull as mapping from old to new indices in mesh.
        // And generated triangles
        var hullVertices = {};
        hullVertices[i0] = 0;
        hullVertices[i1] = 1;
        var outVertexCount = 2;

        var hullTriangles = [];

        while (openSet.length > 0)
        {
            // [ ...., i0, i1 ]
            i1 = openSet.removeLast();
            i0 = openSet.removeLast();

            // (i0 + ":" + i1) in closedSet
            if (closedSet.containsKey('$i0:$i1'))
            {
                continue;
            }

            var i2 = -1;
            var maxEdge0, maxEdge1, maxEdge2;
            var maxDistance, maxProjection;

            p00 = points[i0 * 3];
            p01 = points[(i0 * 3) + 1];
            p02 = points[(i0 * 3) + 2];
            var edge0 = (points[i1 * 3] - p00);
            var edge1 = (points[(i1 * 3) + 1] - p01);
            var edge2 = (points[(i1 * 3) + 2] - p02);
            var isq = 1 / ((edge0 * edge0) + (edge1 * edge1) * (edge2 * edge2));

            for (i = 0; i < (numPoints * 3); i += 3)
            {
                if (i == (i0 * 3) || i == (i1 * 3))
                {
                    continue;
                }

                p0 = points[i];
                p1 = points[i + 1];
                p2 = points[i + 2];

                // Find closest point on line containing the edge to determine vector to p
                // perpendicular to edge. This is not necessary for computing the turn
                // since the value of 'turn' computed is actually the same whether we do
                // this or not, however it is needed to be able to sort equal turn vertices
                // by distance.
                var t = (((p0 - p00) * edge0) +
                         ((p1 - p01) * edge1) +
                         ((p2 - p02) * edge2)) * isq;
                var pEdge0 = (p0 - (p00 + (edge0 * t)));
                var pEdge1 = (p1 - (p01 + (edge1 * t)));
                var pEdge2 = (p2 - (p02 + (edge2 * t)));

                // Ignore vertex if |pedge| = 0, thus ignoring vertices on edge itself
                // and so avoiding generating degenerate triangles.
                var plsq = ((pEdge0 * pEdge0) + (pEdge1 * pEdge1) + (pEdge2 * pEdge2));
                if (plsq <= WebGLPhysicsConfig.COLLINEAR_THRESHOLD)
                {
                    continue;
                }

                if (i2 == -1)
                {
                    i2 = (i / 3);
                    maxEdge0 = pEdge0;
                    maxEdge1 = pEdge1;
                    maxEdge2 = pEdge2;
                    maxDistance = plsq;
                    maxProjection = t;
                    continue;
                }

                // If this is not the first vertex tested, determine if new vertex
                // is a right turn looking in direction of edge, or is further in
                // same direction
                //
                // We require a special case when pedge, and maxedge are coplanar
                // with edge as the computed turn will be 0 and we must check
                // if the cross product is facing into the hull or outside to
                // determine left/right instead.
                var axis0 = ((pEdge1 * maxEdge2) - (pEdge2 * maxEdge1));
                var axis1 = ((pEdge2 * maxEdge0) - (pEdge0 * maxEdge2));
                var axis2 = ((pEdge0 * maxEdge1) - (pEdge1 * maxEdge0));

                var coplanar = (pEdge0 * ((edge1 * maxEdge2) - (edge2 * maxEdge1)) +
                                pEdge1 * ((edge2 * maxEdge0) - (edge0 * maxEdge2)) +
                                pEdge2 * ((edge0 * maxEdge1) - (edge1 * maxEdge0)));
                if ((coplanar * coplanar) < WebGLPhysicsConfig.COPLANAR_THRESHOLD)
                {
                    // Special case for coplanar pedge, maxpedge, edge
                    //
                    // If edges are in same direction, base on distance
                    if (((pEdge0 * maxEdge0) + (pEdge1 * maxEdge1) + (pEdge2 * maxEdge2)) >= 0)
                    {
                        if (plsq > maxDistance || (plsq == maxDistance && t > maxProjection))
                        {
                            i2 = (i / 3);
                            maxEdge0 = pEdge0;
                            maxEdge1 = pEdge1;
                            maxEdge2 = pEdge2;
                            maxDistance = plsq;
                            maxProjection = t;
                        }
                    }
                    else
                    {
                        d0 = (p0 - p00);
                        d1 = (p1 - p01);
                        var d2 = (p2 - p02);
                        axis0 = ((d1 * edge2) - (d2 * edge1));
                        axis1 = ((d2 * edge0) - (d0 * edge2));
                        axis2 = ((d0 * edge1) - (d1 * edge0));

                        // Determine if axis points into, or out of the convex hull.
                        var internal = true;
                        var j;
                        for (j = 0; j < (numPoints * 3); j += 3)
                        {
                            if (((axis0 * (points[j] - p00)) +
                                 (axis1 * (points[j + 1] - p01)) +
                                 (axis2 * (points[j + 2] - p02))) < 0)
                            {
                                internal = false;
                                break;
                            }
                        }

                        if (internal)
                        {
                            i2 = (i / 3);
                            maxEdge0 = pEdge0;
                            maxEdge1 = pEdge1;
                            maxEdge2 = pEdge2;
                            maxDistance = plsq;
                            maxProjection = t;
                        }
                    }
                }
                else
                {
                    var turn = (axis0 * edge0) + (axis1 * edge1) + (axis2 * edge2);
                    if (turn < 0 || (turn <= WebGLPhysicsConfig.COLLINEAR_THRESHOLD && plsq > maxDistance))
                    {
                        i2 = (i / 3);
                        maxEdge0 = pEdge0;
                        maxEdge1 = pEdge1;
                        maxEdge2 = pEdge2;
                        maxDistance = plsq;
                        maxProjection = t;
                    }
                }
            }

            // append i2 vertex to hull
            if (!(hullVertices.containsKey(i2))) {
                hullVertices[i2] = outVertexCount;
                outVertexCount += 1;
            }

            // form triangle iff no edge is closed
            /*if (!((i0 + ":" + i1) in closedSet ||
                  (i1 + ":" + i2) in closedSet ||
                  (i2 + ":" + i0) in closedSet))*/
            if (!closedSet.containsKey('$i0:$i1') ||
                closedSet.containsKey('$i1:$i2') ||
                closedSet.containsKey('$i2:$i0'))
            {
                hullTriangles.add(i0);
                hullTriangles.add(i1);
                hullTriangles.add(i2);

                closedSet[i0 + ":" + i1] = true;
                closedSet[i1 + ":" + i2] = true;
                closedSet[i2 + ":" + i0] = true;

                openSet.add(i2);
                openSet.add(i1);
                openSet.add(i0);
                openSet.add(i2);
            }
        }

        // Output triangle array!
        return createArray(points, hullTriangles, hullVertices, outVertexCount);
    }

    static WebGLPhysicsPrivateTriangleArray createArray(List<double> points, List<int> indices, Map mapping, int vertexCount) {
      // Port removeRedundantVertices from mesh.py with extra param to specify used vertices.
      // Modified to create a WebGLPhysicsPrivateTriangleArray
      var outPoints = new Float32List(vertexCount * 3);
      var triangleCount = indices.length;
      var outIndices = (vertexCount < 65536 ? new Uint16List(triangleCount) : new Uint32List(triangleCount));

      // Produce outPoints array
      var numPoints = (points.length / 3);
      var i;
      for (i = 0; i < numPoints; i += 1)
      {
          if (!(mapping.containsKey(i)))
          {
              continue;
          }

          var newIndex = (mapping[i] * 3);
          outPoints[newIndex] = points[i * 3];
          outPoints[newIndex + 1] = points[(i * 3) + 1];
          outPoints[newIndex + 2] = points[(i * 3) + 2];
      }

      // Remap triangles.
      for (i = 0; i < triangleCount; i += 1)
      {
          outIndices[i] = mapping[indices[i]];
      }
      // Inform TriangleArray constructor not to take a copy of mesh.
      return new WebGLPhysicsTriangleArray(outPoints,outIndices,true); //._private;
    }
}