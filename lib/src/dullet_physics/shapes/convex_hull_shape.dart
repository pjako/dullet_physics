part of dullet_physics;



//
// WebGL Physics Convex Hull Shape
//
class WebGLPhysicsConvexHullShape extends WebGLPhysicsShape
 {
  static const int version = 1;

  //String type = "CONVEX_HULL";
  //double margin;
  //double radius;
  //Vector3 halfExtents; // v3
  //Vector3 inertia; // v3

  //WebGLPhysicsShape _public;
  //Vector3 center; // v3
  //double collisionRadius;
  Float32List points;

  WebGLPhysicsPrivateTriangleArray triangleArray;
  List<int> supportTopology; // Int32Array / Int16Array

  int lastSupport;


  WebGLPhysicsConvexHullShape({
  double margin: 0.04, 
  Float32List points}
  ) {
    //var retc = new PhysicsShape();
    var c = this;
    //retc._private = c;
    //c._public = retc;

    //var margin = (params.margin != null) ? params.margin : 0.04;
    //var points = params.points;

    var min0 = points[0];
    var min1 = points[1];
    var min2 = points[2];
    var max0 = min0;
    var max1 = min1;
    var max2 = min2;
    var maxN = points.length;
    var n;
    var v0, v1, v2;
    for (n = 3; n < maxN; n += 3) {
      v0 = points[n];
      v1 = points[n + 1];
      v2 = points[n + 2];
      if (min0 > v0) {
        min0 = v0;
      } else {
        if (max0 < v0) {
          max0 = v0;
        }
      }
      if (min1 > v1) {
        min1 = v1;
      } else {
        if (max1 < v1) {
          max1 = v1;
        }
      }
      if (min2 > v2) {
        min2 = v2;
      } else {
        if (max2 < v2) {
          max2 = v2;
        }
      }
    }

    var h0 = ((0.5 * (max0 - min0)) + margin);
    var h1 = ((0.5 * (max1 - min1)) + margin);
    var h2 = ((0.5 * (max2 - min2)) + margin);
    var c0 = (0.5 * (min0 + max0));
    var c1 = (0.5 * (min1 + max1));
    var c2 = (0.5 * (min2 + max2));

    var lx = (2.0 * h0);
    var ly = (2.0 * h1);
    var lz = (2.0 * h2);
    lx *= lx;
    ly *= ly;
    lz *= lz;

    var massRatio = (1.0 / 12.0);

    this.points = new Float32List.fromList(points);
    _radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
    _halfExtents = new Vector3(h0, h1, h2);
    if (c0 != 0.0 || c1 != 0.0 || c2 != 0.0) {
      _center = new Vector3(c0, c1, c2);
    } else {
      _center = null;
    }
    _inertia = new Vector3(massRatio * (ly + lz),
     massRatio * (lx + lz),
     massRatio * (lx + ly));
    _collisionRadius = margin;

    // Generate triangle array for ray testing
    if (points.length < 9) {
      // Less than 3 points... cannot generate triangles.
      throw "At present time, WebGL PhysicsDevice does not permit a convex hull to contain " +
       "less than 3 vertices";
    } else {
      bool planar = WebGLPhysicsConvexHullHelpers.isPlanar(points);
      if (planar) {
        c.triangleArray = WebGLPhysicsConvexHullHelpers.makePlanarConvexHull(points);
      } else {
        c.triangleArray = WebGLPhysicsConvexHullHelpers.makeConvexHull(points);
      }

      // Produce edge topology for faster support search.  Each
      // vertex keeps a reference to each neighbouring vertex on
      // triangulated surface.
      //
      // Experiment showed that even for a planar convex hull of
      // 3 vertices, we only search on average 1.6 vertices
      // instead of all 3 so is better even for this smallest
      // case.
      var supportTopology = [];

      points = c.triangleArray.vertices;
      maxN = points.length;
      for (n = 0; n < maxN; n += 3) {
        supportTopology[n / 3] = [];
      }

      var m;
      if (planar) {
        for (n = 0; n < (maxN / 3); n += 1) {
          m = (n + 1) % (maxN / 3);
          supportTopology[n].push(m);
          supportTopology[m].push(n);
        }
      } else {
        var triangles = c.triangleArray.indices;
        maxN = triangles.length;
        for (n = 0; n < maxN; n += 3) {
          var i0 = triangles[n];
          var i1 = triangles[n + 1];
          var i2 = triangles[n + 2];

          // Create links i0 -> i1 -> i2 -> i0.
          // Adjacent triangles will take care of back links.
          supportTopology[i0].push(i1);
          supportTopology[i1].push(i2);
          supportTopology[i2].push(i0);
        }
      }

      // Additionally each vertex keeps a reference to the vertex on far side of hull.
      // Tests show that this reduces the number of vertices searched in support function.
      //
      // Planar case, only do this for 6 vertices or more, or else includes uneeded edges.
      // Non-planar case, do this for 10 vertices or more. Experiment showed this to be a good
      // threshold.
      maxN = points.length;
      if ((planar && maxN >= (3 * 6)) || (!planar && maxN >= (3 * 10))) {
        for (n = 0; n < maxN; n += 3) {
          var min = double.MAX_FINITE;//Number.MAX_VALUE;
          v0 = points[n];
          v1 = points[n + 1];
          v2 = points[n + 2];

          var minm;
          for (m = 0; m < maxN; m += 3) {
            var dot = (v0 * points[m]) + (v1 * points[m + 1]) + (v2 * points[m + 2]);
            if (dot < min) {
              min = dot;
              minm = m;
            }
          }

          supportTopology[n / 3].push(minm / 3);
        }
      }

      // Flatten supportTopology array of arrays, into a single typed array.
      //
      // We take topology array like: [[1,2],[0],[0,1,3],[0,1,2]]
      // Decorate with index of vertex in triangle array
      // and number of edges: [[0,2|1,2], [3,1|0], [6,3|0,1,3], [9,3|0,1,2]]
      // And then flatten into array: [0,2,4,7, 3,1,0, 6,3,0,4,12, 9,3,0,4,7]
      //

      // Compute size of array, and positions of vertex data.
      var mapping = [];
      var size = 0;
      for (n = 0; n < (maxN / 3); n += 1) {
        mapping.add(size);
        size += supportTopology[n].length + 2;
      }

      // Produce flattened array.
      c.supportTopology = (size > 65536) ? new Int32List(size) : new Int16List(size);
      var index = 0;
      for (n = 0; n < (maxN / 3); n += 1) {
        c.supportTopology[index] = (n * 3);
        index += 1;

        var topology = supportTopology[n];
        c.supportTopology[index] = topology.length;
        index += 1;

        for (m = 0; m < topology.length; m += 1) {
          c.supportTopology[index] = mapping[topology[m]];
          index += 1;
        }
      }
    }

  //initShapeProperties(this, "CONVEX_HULL");
  //return retc;
  }

  RayHit rayTest(Ray ray)
   {
    var triangleArray = this.triangleArray;
    if (triangleArray == null)
     {
      return null;
    }

    return triangleArray.rayTest(ray);
  }

  void localSupportWithoutMargin(Vector3 vec, Vector3 dst) {
    var v0 = vec.storage[0];
    var v1 = vec.storage[1];
    var v2 = vec.storage[2];

    var topology = this.supportTopology;
    var points = this.triangleArray.vertices;
    if (this.lastSupport == null)
     {
      this.lastSupport = 0;
    }

    // Start search at last support point.
    var maxv = this.lastSupport;
    var ind = topology[maxv];
    var max = (points[ind] * v0) + (points[ind + 1] * v1) + (points[ind + 2] * v2);

    for (; ;)
     {
      // Determine if a vertex linked in topology is a better support point.
      var next = -1;
      var n;
      var maxN = topology[maxv + 1];
      for (n = 0; n < maxN; n += 1)
       {
        var v = topology[maxv + 2 + n];
        ind = topology[v];
        var vdot = (points[ind] * v0) + (points[ind + 1] * v1) + (points[ind + 2] * v2);
        if (vdot > max)
         {
          max = vdot;
          next = v;
        }
      }

      // If no better support was found, we are at the maximum support.
      if (next != -1)
       {
        maxv = next;
        continue;
      }
       else
       {
        break;
      }
    }

    // Cache maximum support to seed next call to method.
    this.lastSupport = maxv;

    ind = topology[maxv];
    dst.storage[0] = points[ind];
    dst.storage[1] = points[ind + 1];
    dst.storage[2] = points[ind + 2];
  }

}

//WebGLPhysicsConvexHullShape.prototype.type = ;
