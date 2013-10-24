part of dullet_physics;

class WebGLPhysicsTriangleArray extends WebGLPhysicsPrivateTriangleArray /*implements PhysicsTriangleArray*/ {

    //static version = 1;

    // PhysicsTriangleArray
  //Float32List vertices; // getter for _private.vertices
  //Uint16List indices;          // Uint16Array / Uint32Array

  //WebGLPhysicsPrivateTriangleArray _private;

  WebGLPhysicsTriangleArray(List<double> inVertices, List<int> inIndices, bool dontCopy,{ var minExtent, var maxExtent}) {
    //var rett = this;//new WebGLPhysicsTriangleArray();
    var t = this;
    //rett._private = t;
    //t._public = rett;

    var vertices = inVertices;
    var numVertices = (vertices.length / 3);
    var indices = inIndices;
    var numTriangles = (indices.length / 3);

    //var minExtent = minExtent;
    //var maxExtent = maxExtent;

    var v0;
    var v1;
    var v2;

    if (!minExtent || !maxExtent)
    {
      var min0 = vertices[0];
      var min1 = vertices[1];
      var min2 = vertices[2];
      var max0 = min0;
      var max1 = min1;
      var max2 = min2;
      var maxN = vertices.length;
      for (var n = 3; n < maxN; n += 3)
      {
        v0 = vertices[n];
        v1 = vertices[n + 1];
        v2 = vertices[n + 2];
        if (min0 > v0)
        {
          min0 = v0;
        }
        else if (max0 < v0)
        {
          max0 = v0;
        }
        if (min1 > v1)
        {
          min1 = v1;
        }
        else if (max1 < v1)
        {
          max1 = v1;
        }
        if (min2 > v2)
        {
          min2 = v2;
        }
        else if (max2 < v2)
        {
          max2 = v2;
        }
      }
      minExtent = [min0, min1, min2];
      maxExtent = [max0, max1, max2];
    }

    var extents = new Float32List(6);
    extents[0] = minExtent[0];
    extents[1] = minExtent[1];
    extents[2] = minExtent[2];
    extents[3] = maxExtent[0];
    extents[4] = maxExtent[1];
    extents[5] = maxExtent[2];

    t.vertices = (dontCopy ? vertices : new Float32List.fromList(vertices));
    t.numVertices = numVertices;
    t.indices = (dontCopy ? indices : (numVertices < 65536 ? new Uint16List.fromList(indices) : new Uint32List.fromList(indices)));
    t.numTriangles = numTriangles;
    t.extents = extents;

    // read only, no getter needed.
    /*
    Object.defineProperty(rett, "vertices", {
      value : t.vertices,
      enumerable : true
    });
    Object.defineProperty(rett, "indices", {
      value : t.indices,
      enumerable : true
    });*/

    /*
    store pre-computed triangle information for ray tests

    n0 n1 n2 - triangle normal
    v0 v1 v2 - triangle vertex
    u0 u1 u2 v0 v1 v2  - edge vectors
    dotuu dotvv dotuv negLimit - barycentric constants
    d - triangle plane distance
    */
    var triangles = new Float32List(WebGLPhysicsTriangleArray.TRIANGLE_SIZE * numTriangles);
    var spatialMap = null;

    // Only use spatial map if we do not have a trivial number of triangles.
    if (numTriangles >= 8)
    {
      spatialMap = new AabbTree(true);
    }

    var i;
    for (i = 0; i < numTriangles; i = i + 1) {
      var i3 = (i * 3);
      var itri = (i * WebGLPhysicsTriangleArray.TRIANGLE_SIZE);

      var i0 = (indices[i3] * 3);
      var i1 = (indices[i3 + 1] * 3);
      var i2 = (indices[i3 + 2] * 3);

      var v00 = vertices[i0];
      var v01 = vertices[i0 + 1];
      var v02 = vertices[i0 + 2];

      var v10 = vertices[i1];
      var v11 = vertices[i1 + 1];
      var v12 = vertices[i1 + 2];

      var v20 = vertices[i2];
      var v21 = vertices[i2 + 1];
      var v22 = vertices[i2 + 2];

      //var u = VMath.v3Sub(v1, v0);
      //var v = VMath.v3Sub(v2, v0);
      var u0 = (v10 - v00);
      var u1 = (v11 - v01);
      var u2 = (v12 - v02);
      v0 = (v20 - v00);
      v1 = (v21 - v01);
      v2 = (v22 - v02);

      //var normal = VMath.v3Cross(u, v);
      var n0 = ((u1 * v2) - (u2 * v1));
      var n1 = ((u2 * v0) - (u0 * v2));
      var n2 = ((u0 * v1) - (u1 * v0));
      var nn = (1.0 / Math.sqrt((n0 * n0) + (n1 * n1) + (n2 * n2)));

      var distance = (((n0 * v00) + (n1 * v01) + (n2 * v02)) * nn);

      //var dotuv = VMath.v3Dot(u, v);
      //var dotuu = VMath.v3Dot(u, u);
      //var dotvv = VMath.v3Dot(v, v);
      var dotuv = ((u0 * v0) + (u1 * v1) + (u2 * v2));
      var dotuu = ((u0 * u0) + (u1 * u1) + (u2 * u2));
      var dotvv = ((v0 * v0) + (v1 * v1) + (v2 * v2));

      // Always negative
      var negLimit = ((dotuv * dotuv) - (dotuu * dotvv));

      triangles[itri] = (n0 * nn);
      triangles[itri + 1] = (n1 * nn);
      triangles[itri + 2] = (n2 * nn);
      triangles[itri + 3] = v00;
      triangles[itri + 4] = v01;
      triangles[itri + 5] = v02;
      triangles[itri + 6] = u0;
      triangles[itri + 7] = u1;
      triangles[itri + 8] = u2;
      triangles[itri + 9] = v0;
      triangles[itri + 10] = v1;
      triangles[itri + 11] = v2;
      triangles[itri + 12] = dotuu;
      triangles[itri + 13] = dotvv;
      triangles[itri + 14] = dotuv;
      triangles[itri + 15] = negLimit;
      triangles[itri + 16] = distance;

      // If building AABBTree, store node
      if (spatialMap)
      {
        extents = new Float32List(6);
        extents[0] = MathHelper.min(v00, v10, v20);
        extents[1] = MathHelper.min(v01, v11, v21);
        extents[2] = MathHelper.min(v02, v12, v22);
        extents[3] = MathHelper.max(v00, v10, v20);
        extents[4] = MathHelper.max(v01, v11, v21);
        extents[5] = MathHelper.max(v02, v12, v22);

        var triNode = {
                       "index": itri
        };
        spatialMap.add(triNode, extents);
      }
    }

    if (spatialMap)
    {
      spatialMap.finalize();
    }

    t.triangles = triangles;
    t.spatialMap = spatialMap;

    //return rett;

  }

}
/*
static WebGLPhysicsTriangleArray create(params) {
var rett = new WebGLPhysicsTriangleArray();
var t = new WebGLPhysicsPrivateTriangleArray();
rett._private = t;
t._public = rett;

var vertices = params.vertices;
var numVertices = (vertices.length / 3);
var indices = params.indices;
var numTriangles = (indices.length / 3);

var minExtent = params.minExtent;
var maxExtent = params.maxExtent;

var v0;
var v1;
var v2;

if (!minExtent || !maxExtent)
{
var min0 = vertices[0];
var min1 = vertices[1];
var min2 = vertices[2];
var max0 = min0;
var max1 = min1;
var max2 = min2;
var maxN = vertices.length;
for (var n = 3; n < maxN; n += 3)
{
v0 = vertices[n];
v1 = vertices[n + 1];
v2 = vertices[n + 2];
if (min0 > v0)
{
min0 = v0;
}
else if (max0 < v0)
{
max0 = v0;
}
if (min1 > v1)
{
min1 = v1;
}
else if (max1 < v1)
{
max1 = v1;
}
if (min2 > v2)
{
min2 = v2;
}
else if (max2 < v2)
{
max2 = v2;
}
}
minExtent = [min0, min1, min2];
maxExtent = [max0, max1, max2];
}

var extents = new Float32List(6);
extents[0] = minExtent[0];
extents[1] = minExtent[1];
extents[2] = minExtent[2];
extents[3] = maxExtent[0];
extents[4] = maxExtent[1];
extents[5] = maxExtent[2];

t.vertices = (params.dontCopy ? vertices : new Float32List.fromList(vertices));
t.numVertices = numVertices;
t.indices = (params.dontCopy ? indices : (numVertices < 65536 ? new Uint16List.fromList(indices) : new Uint32List.fromList(indices)));
t.numTriangles = numTriangles;
t.extents = extents;

// read only, no getter needed.
/*
Object.defineProperty(rett, "vertices", {
value : t.vertices,
enumerable : true
});
Object.defineProperty(rett, "indices", {
value : t.indices,
enumerable : true
});*/

/*
store pre-computed triangle information for ray tests

n0 n1 n2 - triangle normal
v0 v1 v2 - triangle vertex
u0 u1 u2 v0 v1 v2  - edge vectors
dotuu dotvv dotuv negLimit - barycentric constants
d - triangle plane distance
 */
var triangles = new Float32List(WebGLPhysicsTriangleArray.TRIANGLE_SIZE * numTriangles);
var spatialMap = null;

// Only use spatial map if we do not have a trivial number of triangles.
if (numTriangles >= 8)
{
spatialMap = AABBTree.create(true);
}

var i;
for (i = 0; i < numTriangles; i = i + 1)
{
var i3 = (i * 3);
var itri = (i * WebGLPhysicsTriangleArray.TRIANGLE_SIZE);

var i0 = (indices[i3] * 3);
var i1 = (indices[i3 + 1] * 3);
var i2 = (indices[i3 + 2] * 3);

var v00 = vertices[i0];
var v01 = vertices[i0 + 1];
var v02 = vertices[i0 + 2];

var v10 = vertices[i1];
var v11 = vertices[i1 + 1];
var v12 = vertices[i1 + 2];

var v20 = vertices[i2];
var v21 = vertices[i2 + 1];
var v22 = vertices[i2 + 2];

//var u = VMath.v3Sub(v1, v0);
//var v = VMath.v3Sub(v2, v0);
var u0 = (v10 - v00);
var u1 = (v11 - v01);
var u2 = (v12 - v02);
v0 = (v20 - v00);
v1 = (v21 - v01);
v2 = (v22 - v02);

//var normal = VMath.v3Cross(u, v);
var n0 = ((u1 * v2) - (u2 * v1));
var n1 = ((u2 * v0) - (u0 * v2));
var n2 = ((u0 * v1) - (u1 * v0));
var nn = (1.0 / Math.sqrt((n0 * n0) + (n1 * n1) + (n2 * n2)));

var distance = (((n0 * v00) + (n1 * v01) + (n2 * v02)) * nn);

//var dotuv = VMath.v3Dot(u, v);
//var dotuu = VMath.v3Dot(u, u);
//var dotvv = VMath.v3Dot(v, v);
var dotuv = ((u0 * v0) + (u1 * v1) + (u2 * v2));
var dotuu = ((u0 * u0) + (u1 * u1) + (u2 * u2));
var dotvv = ((v0 * v0) + (v1 * v1) + (v2 * v2));

// Always negative
var negLimit = ((dotuv * dotuv) - (dotuu * dotvv));

triangles[itri] = (n0 * nn);
triangles[itri + 1] = (n1 * nn);
triangles[itri + 2] = (n2 * nn);
triangles[itri + 3] = v00;
triangles[itri + 4] = v01;
triangles[itri + 5] = v02;
triangles[itri + 6] = u0;
triangles[itri + 7] = u1;
triangles[itri + 8] = u2;
triangles[itri + 9] = v0;
triangles[itri + 10] = v1;
triangles[itri + 11] = v2;
triangles[itri + 12] = dotuu;
triangles[itri + 13] = dotvv;
triangles[itri + 14] = dotuv;
triangles[itri + 15] = negLimit;
triangles[itri + 16] = distance;

// If building AABBTree, store node
if (spatialMap)
{
extents = new Float32List(6);
extents[0] = Math.min(v00, v10, v20);
extents[1] = Math.min(v01, v11, v21);
extents[2] = Math.min(v02, v12, v22);
extents[3] = Math.max(v00, v10, v20);
extents[4] = Math.max(v01, v11, v21);
extents[5] = Math.max(v02, v12, v22);

var triNode = {
"index": itri
};
spatialMap.add(triNode, extents);
}
}

if (spatialMap)
{
spatialMap.finalize();
}

t.triangles = triangles;
t.spatialMap = spatialMap;

return rett;
}*/

class WebGLPhysicsPrivateTriangleArray {
    //static version = 1;

    // Size of each 'triangle' in triangles array.
    static const TRIANGLE_SIZE = 17; // prototype

    //WebGLPhysicsTriangleArray _public;
    double numVertices;
    Float32List extents;
    Float32List vertices;
    Uint16List indices;   // Uint16Array / Uint32Array
    Float32List triangles;
    int numTriangles;
    AabbTree spatialMap;

    rayTest(ray) {
        var triangles = this.triangles;
        var spatialMap = this.spatialMap;

        RayHit rayCallback(tree, triangle, ray, unusedAABBDistance, upperBound) {
            var dir = ray.direction;
            var dir0 = dir[0];
            var dir1 = dir[1];
            var dir2 = dir[2];

            var origin = ray.origin;
            var o0 = origin[0];
            var o1 = origin[1];
            var o2 = origin[2];

            var i = triangle.index;
            var n0 = triangles[i];
            var n1 = triangles[i + 1];
            var n2 = triangles[i + 2];

            //var dot = VMath.v3Dot(ray.direction, normal);
            var dot = ((dir0 * n0) + (dir1 * n1) + (dir2 * n2));
            // If ray is parallel to triangle plane
            // Assume it cannot intersect triangle
            if ((dot * dot) < WebGLPhysicsConfig.COPLANAR_THRESHOLD)
            {
                return null;
            }

            var d = triangles[i + 16];
            var v00 = triangles[i + 3];
            var v01 = triangles[i + 4];
            var v02 = triangles[i + 5];
            //var distance = VMath.v3Dot(VMath.v3Sub(v0, ray.origin), normal) / dot;
            var distance = ((d - ((o0 * n0) + (o1 * n1) + (o2 * n2))) / dot);
            if (distance < 0 || distance >= upperBound)
            {
                return null;
            }

            // Make sure normal points correct direction for ray cast result
            if (dot > 0)
            {
                //normal = VMath.v3Neg(normal);
                n0 = -n0;
                n1 = -n1;
                n2 = -n2;

                dot = -dot;
            }

            //var hitPoint = VMath.v3Add(ray.origin, VMath.v3ScalarMul(ray.direction, distance));
            var hit0 = (o0 + (dir0 * distance));
            var hit1 = (o1 + (dir1 * distance));
            var hit2 = (o2 + (dir2 * distance));

            // Compute barycentric coordinates in triangle.
            //var w = VMath.v3Sub(hitPoint, v0);
            var wx = (hit0 - v00);
            var wy = (hit1 - v01);
            var wz = (hit2 - v02);

            var dotuu = triangles[i + 12];
            var dotvv = triangles[i + 13];
            var dotuv = triangles[i + 14];
            var negLimit = triangles[i + 15];

            var u0 = triangles[i + 6];
            var u1 = triangles[i + 7];
            var u2 = triangles[i + 8];
            var v0 = triangles[i + 9];
            var v1 = triangles[i + 10];
            var v2 = triangles[i + 11];
            //var dotwu = VMath.v3Dot(w, u);
            //var dotwv = VMath.v3Dot(w, v);
            var dotwu = (wx * u0) + (wy * u1) + (wz * u2);
            var dotwv = (wx * v0) + (wy * v1) + (wz * v2);

            var alpha = ((dotuv * dotwv) - (dotvv * dotwu));
            if (alpha > 0 || alpha < negLimit)
            {
                return null;
            }

            var beta  = ((dotuv * dotwu) - (dotuu * dotwv));
            if (beta > 0 || (alpha + beta) < negLimit)
            {
                return null;
            }

            return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3(n0, n1, n2), distance);
        }

        if (spatialMap) {
            return AabbTree.rayTest([spatialMap], ray, rayCallback);
        } else {
            var minimumResult = null;
            var upperBound = ray.maxFactor;

            var triNode = {
                "index": 0
            };
            var i;
            var numTris = this.numTriangles * WebGLPhysicsTriangleArray.TRIANGLE_SIZE;
            for (i = 0; i < numTris; i += WebGLPhysicsTriangleArray.TRIANGLE_SIZE)
            {
                triNode['index'] = i;
                var result = rayCallback(null, triNode, ray, 0, upperBound);
                if (result)
                {
                    minimumResult = result;
                    upperBound = minimumResult.factor;
                }
            }

            return minimumResult;
        }
    }
}