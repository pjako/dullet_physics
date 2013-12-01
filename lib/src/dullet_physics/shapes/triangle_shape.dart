part of dullet_physics;


///
/// WebGLPhysicsTriangleShape
///
///
/// Triangle shapes are used for collision detection with triangles
/// of a triangle mesh.  In most cases there is one persistant
/// object used. In the case of continuous collisions we need to
/// reuse many of these objects, so an object pool is used with
/// allocate and deallocate methods instead of a create method.
///
class WebGLPhysicsTriangleShape
 {
  static const version = 1;

  //final String type = "TRIANGLE_MESH_TRIANGLE"; // prototype
  int index;
  double collisionRadius;
  WebGLPhysicsPrivateTriangleArray triangleArray;

  WebGLPhysicsTriangleShape()
   {
    // Initialise all properties of Triangle shape
    // which will ever be used.

    // Index into parent WebGLTriangleArray::triangles list.
    this.index = 0;

    // Collision radius in collision algorithms, this is taken from parent mesh shape.
    this.collisionRadius = 0.0;

    // The parent TriangleArray.
    this.triangleArray = null;

  }

  localSupportWithoutMargin(Float32List vec, Float32List dst) {
    var vec0 = vec[0];
    var vec1 = vec[1];
    var vec2 = vec[2];

    var triangles = this.triangleArray.triangles;
    var triangle = this.index;

    var v00 = triangles[triangle + 3];
    var v01 = triangles[triangle + 4];
    var v02 = triangles[triangle + 5];
    var u0 = triangles[triangle + 6];
    var u1 = triangles[triangle + 7];
    var u2 = triangles[triangle + 8];
    var v0 = triangles[triangle + 9];
    var v1 = triangles[triangle + 10];
    var v2 = triangles[triangle + 11];

    var dotu = ((vec0 * u0) + (vec1 * u1) + (vec2 * u2));
    var dotv = ((vec0 * v0) + (vec1 * v1) + (vec2 * v2));

    if (dotu <= 0 && dotv <= 0) {
      dst[0] = v00;
      dst[1] = v01;
      dst[2] = v02;
    } else {
      if (dotu >= dotv) {
        dst[0] = (v00 + u0);
        dst[1] = (v01 + u1);
        dst[2] = (v02 + u2);
      } else {
        dst[0] = (v00 + v0);
        dst[1] = (v01 + v1);
        dst[2] = (v02 + v2);
      }
    }
  }

  static final List<WebGLPhysicsTriangleShape> trianglePool = <WebGLPhysicsTriangleShape>[];

  static WebGLPhysicsTriangleShape allocate()
   {
    var triangle;
    if (trianglePool.isEmpty) {
      triangle = new WebGLPhysicsTriangleShape();
    } else {
      triangle = trianglePool.removeLast();
    }

    return triangle;
  }

  static void deallocate(WebGLPhysicsTriangleShape triangle) {
    trianglePool.add(triangle);
    // Ensure reference is null'ed so that an object pooled Triangle Shape
    // cannot prevent the TriangleArray from being GC'ed.

    triangle.triangleArray = null;
  }
}
