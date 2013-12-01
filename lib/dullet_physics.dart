library dullet_physics;
import 'dart:math' as Math;
import 'dart:typed_data';
import 'package:vector_math/vector_math.dart';

part 'src/dullet_physics/arbiter.dart';
part 'src/dullet_physics/device.dart';
part 'src/dullet_physics/globals.dart';
part 'src/dullet_physics/island.dart';
part 'src/dullet_physics/private_body.dart';
part 'src/dullet_physics/world.dart';

part 'src/dullet_physics/collision/character.dart';
part 'src/dullet_physics/collision/collision_object.dart';
part 'src/dullet_physics/collision/contact.dart';
part 'src/dullet_physics/collision/contact_callbacks.dart';
part 'src/dullet_physics/collision/contact_epa.dart';
part 'src/dullet_physics/collision/rigid_body.dart';

part 'src/dullet_physics/constraints/constraint.dart';
part 'src/dullet_physics/constraints/point2point_constraint.dart';

part 'src/dullet_physics/helpers/aabb_tree.dart';
part 'src/dullet_physics/helpers/convex_hull_helpers.dart';
part 'src/dullet_physics/helpers/math_helper.dart';

part 'src/dullet_physics/shapes/box_shape.dart';
part 'src/dullet_physics/shapes/capsule_shape.dart';
part 'src/dullet_physics/shapes/cone_shape.dart';
part 'src/dullet_physics/shapes/convex_hull_shape.dart';
part 'src/dullet_physics/shapes/cylinder_shape.dart';
part 'src/dullet_physics/shapes/plane_shape.dart';
part 'src/dullet_physics/shapes/shape.dart';
part 'src/dullet_physics/shapes/sphere_shape.dart';
part 'src/dullet_physics/shapes/triangle_shape.dart';
part 'src/dullet_physics/shapes/triangle_array.dart';
part 'src/dullet_physics/shapes/triangle_mesh_shape.dart';
part 'src/dullet_physics/solver/gjk_contact_solver.dart';

class WebGLPhysicsPerformance {
  double discrete = 0.0;
  double sleepComputation = 0.0;
  double prestepContacts = 0.0;
  double prestepConstraints = 0.0;
  double integrateVelocities = 0.0;
  double warmstartContacts = 0.0;
  double warmstartConstraints = 0.0;
  double physicsIterations = 0.0;
  double integratePositions = 0.0;
  double continuous = 0.0;
}

class WebGLPhysicsNarrowCache extends WebGLPhysicsTOIEvent {
  //Vector3 axis = new Vector3.zero(); // v3
  //WebGLPhysicsShape shapeA; // TODO: what is this
  //WebGLPhysicsShape shapeB; // TODO: what is this
  //Vector3 closestA = new Vector3.zero(); // v3
  //Vector3 closestB = new Vector3.zero(); // v3
  WebGLPhysicsNarrowCache() {
    axis = new Vector3.zero(); // v3
    closestA = new Vector3.zero(); // v3
    closestB = new Vector3.zero(); // v3
  }
  WebGLPhysicsNarrowCache.fromData(
  Vector3 axis, 
  WebGLPhysicsShape shapeA, 
  WebGLPhysicsShape shapeB, 
  Vector3 closestA, 
  Vector3 closestB) {
    this.axis = axis;
    this.shapeA = shapeA;
    this.shapeB = shapeB;
    this.closestA = closestA;
    this.closestB = closestB;
  }

}
class FakeBody {
  WebGLPhysicsShape _shape;
  Matrix43 _transform = new Matrix43();
  Matrix43 _startTransform = new Matrix43();
}


class Ray {
  Ray(this.origin, this.direction, this.maxFactor);
  Vector3 origin;
  Vector3 direction;
  double maxFactor;
//int mask;
}



class Matrix43 {
  final Float32List storage = new Float32List(12);

  Matrix43();


  Matrix43.fromTranslation(double x, double y, double z) {
    storage[0] = 1.0;
    storage[1] = 0.0;
    storage[2] = 0.0;
    storage[3] = 0.0;
    storage[4] = 1.0;
    storage[5] = 0.0;
    storage[6] = 0.0;
    storage[7] = 0.0;
    storage[8] = 1.0;
    storage[9] = x;
    storage[10] = y;
    storage[11] = z;
  }


  Matrix43.identity() {
    storage[0] = 1.0;
    storage[1] = 0.0;
    storage[2] = 0.0;
    storage[3] = 0.0;
    storage[4] = 1.0;
    storage[5] = 0.0;
    storage[6] = 0.0;
    storage[7] = 0.0;
    storage[8] = 1.0;
    storage[9] = 0.0;
    storage[10] = 0.0;
    storage[11] = 0.0;
  }

  Matrix43 clone() {
    return copyInto(new Matrix43());
  }

  Matrix43 copyInto(Matrix43 dest) {
    dest.storage[0] = storage[0];
    dest.storage[1] = storage[1];
    dest.storage[2] = storage[2];
    dest.storage[3] = storage[3];
    dest.storage[4] = storage[4];
    dest.storage[5] = storage[5];
    dest.storage[6] = storage[6];
    dest.storage[7] = storage[7];
    dest.storage[8] = storage[8];
    dest.storage[9] = storage[9];
    dest.storage[10] = storage[10];
    dest.storage[11] = storage[11];
    return dest;
  }

  Matrix43 inverseOrthonormal(Matrix43 dest) {
    var m0 = storage[0];
    var m1 = storage[1];
    var m2 = storage[2];
    var m3 = storage[3];
    var m4 = storage[4];
    var m5 = storage[5];
    var m6 = storage[6];
    var m7 = storage[7];
    var m8 = storage[8];
    var px = storage[9];
    var py = storage[10];
    var pz = storage[11];
    dest.storage[0] = m0;
    dest.storage[1] = m3;
    dest.storage[2] = m6;
    dest.storage[3] = m1;
    dest.storage[4] = m4;
    dest.storage[5] = m7;
    dest.storage[6] = m2;
    dest.storage[7] = m5;
    dest.storage[8] = m8;
    dest.storage[9] = -((px * m0) + (py * m1) + (pz * m2));
    dest.storage[10] = -((px * m3) + (py * m4) + (pz * m5));
    dest.storage[11] = -((px * m6) + (py * m7) + (pz * m8));
    return dest;
  }
  Matrix43 multiply(Matrix43 other, Matrix43 dest) {
    var a0 = storage[0];
    var a1 = storage[1];
    var a2 = storage[2];
    var a3 = storage[3];
    var a4 = storage[4];
    var a5 = storage[5];
    var a6 = storage[6];
    var a7 = storage[7];
    var a8 = storage[8];
    var a9 = storage[9];
    var a10 = storage[10];
    var a11 = storage[11];

    var b0 = other.storage[0];
    var b1 = other.storage[1];
    var b2 = other.storage[2];
    var b3 = other.storage[3];
    var b4 = other.storage[4];
    var b5 = other.storage[5];
    var b6 = other.storage[6];
    var b7 = other.storage[7];
    var b8 = other.storage[8];

    dest.storage[0] = (b0 * a0 + b3 * a1 + b6 * a2);
    dest.storage[1] = (b1 * a0 + b4 * a1 + b7 * a2);
    dest.storage[2] = (b2 * a0 + b5 * a1 + b8 * a2);
    dest.storage[3] = (b0 * a3 + b3 * a4 + b6 * a5);
    dest.storage[4] = (b1 * a3 + b4 * a4 + b7 * a5);
    dest.storage[5] = (b2 * a3 + b5 * a4 + b8 * a5);
    dest.storage[6] = (b0 * a6 + b3 * a7 + b6 * a8);
    dest.storage[7] = (b1 * a6 + b4 * a7 + b7 * a8);
    dest.storage[8] = (b2 * a6 + b5 * a7 + b8 * a8);
    dest.storage[9] = (b0 * a9 + b3 * a10 + b6 * a11 + other.storage[9]);
    dest.storage[10] = (b1 * a9 + b4 * a10 + b7 * a11 + other.storage[10]);
    dest.storage[11] = (b2 * a9 + b5 * a10 + b8 * a11 + other.storage[11]);

    return dest;
  }
  Matrix4 multiplyMatrix4(Matrix4 mat, Matrix4 out) {
    var a0 = storage[0];
    var a1 = storage[1];
    var a2 = storage[2];
    var a3 = storage[3];
    var a4 = storage[4];
    var a5 = storage[5];
    var a6 = storage[6];
    var a7 = storage[7];
    var a8 = storage[8];
    var a9 = storage[9];
    var a10 = storage[10];
    var a11 = storage[11];

    var b0 = mat.storage[0];
    var b1 = mat.storage[1];
    var b2 = mat.storage[2];
    var b3 = mat.storage[3];
    var b4 = mat.storage[4];
    var b5 = mat.storage[5];
    var b6 = mat.storage[6];
    var b7 = mat.storage[7];
    var b8 = mat.storage[8];
    var b9 = mat.storage[9];
    var b10 = mat.storage[10];
    var b11 = mat.storage[11];
    out.storage[0] = (b0 * a0 + b4 * a1 + b8 * a2);
    out.storage[1] = (b1 * a0 + b5 * a1 + b9 * a2);
    out.storage[2] = (b2 * a0 + b6 * a1 + b10 * a2);
    out.storage[3] = (b3 * a0 + b7 * a1 + b11 * a2);
    out.storage[4] = (b0 * a3 + b4 * a4 + b8 * a5);
    out.storage[5] = (b1 * a3 + b5 * a4 + b9 * a5);
    out.storage[6] = (b2 * a3 + b6 * a4 + b10 * a5);
    out.storage[7] = (b3 * a3 + b7 * a4 + b11 * a5);
    out.storage[8] = (b0 * a6 + b4 * a7 + b8 * a8);
    out.storage[9] = (b1 * a6 + b5 * a7 + b9 * a8);
    out.storage[10] = (b2 * a6 + b6 * a7 + b10 * a8);
    out.storage[11] = (b3 * a6 + b7 * a7 + b11 * a8);
    out.storage[12] = (b0 * a9 + b4 * a10 + b8 * a11 + mat.storage[12]);
    out.storage[13] = (b1 * a9 + b5 * a10 + b9 * a11 + mat.storage[13]);
    out.storage[14] = (b2 * a9 + b6 * a10 + b10 * a11 + mat.storage[14]);
    out.storage[15] = (b3 * a9 + b7 * a10 + b11 * a11 + mat.storage[15]);
    return out;
  }


  Vector3 inverseOrthonormalTransformPoint(Vector3 v, Vector3 dst) {
    var v0 = v.storage[0] - storage[9];
    var v1 = v.storage[1] - storage[10];
    var v2 = v.storage[2] - storage[11];
    dst.storage[0] = (storage[0] * v0 + storage[1] * v1 + storage[2] * v2);
    dst.storage[1] = (storage[3] * v0 + storage[4] * v1 + storage[5] * v2);
    dst.storage[2] = (storage[6] * v0 + storage[7] * v1 + storage[8] * v2);
    return dst;
  }

  Vector3 inverseOrthonormalTransformVector(Vector3 v, Vector3 dst) {
    var v0 = v.storage[0];
    var v1 = v.storage[1];
    var v2 = v.storage[2];
    dst.storage[0] = (storage[0] * v0 + storage[1] * v1 + storage[2] * v2);
    dst.storage[1] = (storage[3] * v0 + storage[4] * v1 + storage[5] * v2);
    dst.storage[2] = (storage[6] * v0 + storage[7] * v1 + storage[8] * v2);
    return dst;
  }
  Matrix43 negOffset(Vector3 vo, Matrix43 dest) {
    var m = storage;
    var dst = dest.storage;
    var o = vo.storage;
    var m0 = m[0];
    var m1 = m[1];
    var m2 = m[2];
    var m3 = m[3];
    var m4 = m[4];
    var m5 = m[5];
    var m6 = m[6];
    var m7 = m[7];
    var m8 = m[8];
    var m9 = m[9];
    var m10 = m[10];
    var m11 = m[11];

    var o0 = -o[0];
    var o1 = -o[1];
    var o2 = -o[2];

    dst[0] = m0;
    dst[1] = m1;
    dst[2] = m2;
    dst[3] = m3;
    dst[4] = m4;
    dst[5] = m5;
    dst[6] = m6;
    dst[7] = m7;
    dst[8] = m8;
    dst[9] = (m0 * o0 + m3 * o1 + m6 * o2 + m9);
    dst[10] = (m1 * o0 + m4 * o1 + m7 * o2 + m10);
    dst[11] = (m2 * o0 + m5 * o1 + m8 * o2 + m11);

    return dst;
  }
  Vector3 transformPoint(Vector3 v) {
    //var res = dst;
    var m = storage;
    var v0 = v.storage[0];
    var v1 = v.storage[1];
    var v2 = v.storage[2];
    v.storage[0] = (m[0] * v0 + m[3] * v1 + m[6] * v2 + m[9]);
    v.storage[1] = (m[1] * v0 + m[4] * v1 + m[7] * v2 + m[10]);
    v.storage[2] = (m[2] * v0 + m[5] * v1 + m[8] * v2 + m[11]);
    return v;
  }

  Vector3 getPosition(Vector3 pos) {
    var res = pos.storage;
    res[0] = storage[9];
    res[1] = storage[10];
    res[2] = storage[11];
    return res;
  }
  Vector3 get position => new Vector3(storage[9], storage[10], storage[11]);

  double operator [](int i) => storage[i];

  void operator []=(int i, double v) {
    storage[i] = v;
  }


  String toString() =>
   '''[${storage[0]},${storage[1]},${storage[2]}]
[${storage[3]},${storage[4]},${storage[5]}]
[${storage[6]},${storage[7]},${storage[8]}]
[${storage[9]},${storage[10]},${storage[11]}]''';



}





class RayHit {
  double factor;
  Vector3 hitPoint;
  Vector3 hitNormal;
  WebGLPhysicsCollisionObject collisionObject;
  double distance;
  WebGLPhysicsPrivateBody body;
  RayHit(this.hitPoint, this.hitNormal, this.factor);
}


void calculateExtents(FakeBody fakeBody, Aabb3 extents) {
  var shape = fakeBody._shape;
  var center = shape._center;
  var halfExtents = shape._halfExtents;
  var h0 = halfExtents.storage[0];
  var h1 = halfExtents.storage[1];
  var h2 = halfExtents.storage[2];

  var transform = fakeBody._transform;
  var m0 = transform[0];
  var m1 = transform[1];
  var m2 = transform[2];
  var m3 = transform[3];
  var m4 = transform[4];
  var m5 = transform[5];
  var m6 = transform[6];
  var m7 = transform[7];
  var m8 = transform[8];

  var ct0 = transform[9];
  var ct1 = transform[10];
  var ct2 = transform[11];
  if (center)
   {
    var c0 = center[0];
    var c1 = center[1];
    var c2 = center[2];

    if (c0 != 0 ||
     c1 != 0 ||
     c2 != 0)
     {
      ct0 += (m0 * c0 + m3 * c1 + m6 * c2);
      ct1 += (m1 * c0 + m4 * c1 + m7 * c2);
      ct2 += (m2 * c0 + m5 * c1 + m8 * c2);
    }
  }

  // fails when h0, h1, h2 are infinite, as JS has 0 * infinity = NaN not 0!!!!
  //var ht0 = ((m0 < 0 ? -m0 : m0) * h0 + (m3 < 0 ? -m3 : m3) * h1 + (m6 < 0 ? -m6 : m6) * h2);
  //var ht1 = ((m1 < 0 ? -m1 : m1) * h0 + (m4 < 0 ? -m4 : m4) * h1 + (m7 < 0 ? -m7 : m7) * h2);
  //var ht2 = ((m2 < 0 ? -m2 : m2) * h0 + (m5 < 0 ? -m5 : m5) * h1 + (m8 < 0 ? -m8 : m8) * h2);
  var ht0 = ((m0 < 0.0 ? -m0 * h0 : m0 > 0.0 ? m0 * h0 : 0.0) +
   (m3 < 0.0 ? -m3 * h1 : m3 > 0.0 ? m3 * h1 : 0.0) +
   (m6 < 0.0 ? -m6 * h2 : m6 > 0.0 ? m6 * h2 : 0.0));
  var ht1 = ((m1 < 0.0 ? -m1 * h0 : m1 > 0.0 ? m1 * h0 : 0.0) +
   (m4 < 0.0 ? -m4 * h1 : m4 > 0.0 ? m4 * h1 : 0.0) +
   (m7 < 0.0 ? -m7 * h2 : m7 > 0.0 ? m7 * h2 : 0.0));
  var ht2 = ((m2 < 0.0 ? -m2 * h0 : m2 > 0.0 ? m2 * h0 : 0.0) +
   (m5 < 0.0 ? -m5 * h1 : m5 > 0.0 ? m5 * h1 : 0.0) +
   (m8 < 0.0 ? -m8 * h2 : m8 > 0.0 ? m8 * h2 : 0.0));

  extents.min.storage[0] = (ct0 - ht0);
  extents.min.storage[1] = (ct1 - ht1);
  extents.min.storage[2] = (ct2 - ht2);
  extents.max.storage[0] = (ct0 + ht0);
  extents.max.storage[1] = (ct1 + ht1);
  extents.max.storage[2] = (ct2 + ht2);
}


calculateSweptExtents(FakeBody fakeBody, Aabb3 extents) {
  var shape = fakeBody._shape;
  var radius = shape._radius;

  var startTransform = fakeBody._startTransform;
  var x0 = startTransform.storage[9];
  var x1 = startTransform.storage[10];
  var x2 = startTransform.storage[11];

  var transform = fakeBody._transform;
  var y0 = transform.storage[9];
  var y1 = transform.storage[10];
  var y2 = transform.storage[11];

  var tmp;
  if (x0 > y0) {
    tmp = x0;
    x0 = y0;
    y0 = tmp;
  }
  if (x1 > y1) {
    tmp = x1;
    x1 = y1;
    y1 = tmp;
  }
  if (x2 > y2) {
    tmp = x2;
    x2 = y2;
    y2 = tmp;
  }

  extents.min.storage[0] = x0 - radius;
  extents.min.storage[1] = x1 - radius;
  extents.min.storage[2] = x2 - radius;
  extents.max.storage[0] = y0 + radius;
  extents.max.storage[1] = y1 + radius;
  extents.max.storage[2] = y2 + radius;
}



/*
dynamic webGLPhysicsClone(dst, src) {
    for (var p in src)
    {
        if (src.hasOwnProperty(p))
        {
            var v = src[p];
            if (v === null || v === undefined)
            {
                continue;
            }

            if (v is "object" &&
                p !== "shape" &&
                p !== "userData" &&
                p !== "world" &&
                p !== "object" &&
                p !== "arbiters" &&
                p !== "islandRoot" &&
                p !== "island" &&
                p !== "bodyA" &&
                p !== "bodyB" &&
                p !== "triangleArray")
            {
                if ("slice" in v)
                {
                    v = v.slice();
                }
                else
                {
                    v = webGLPhysicsClone({}, v);
                }
            }
            dst[p] = v;
        }
    }
    return dst;
}




var initShapeProperties = function initShapePropertiesFn(s: WebGLPhysicsShape,
                                                         type: string,
                                                         nomargin?: boolean)
{
    // Capsule/Sphere have this defined differently.
    if (!nomargin)
    {
        Object.defineProperty(s, "margin", {
            get : function shapeGetMargin()
            {
                return this._private.collisionRadius;
            },
            set : function shapeSetMargin(margin)
            {
                var pr = this._private;
                pr.halfExtents[0] += (margin - pr.collisionRadius);
                pr.halfExtents[1] += (margin - pr.collisionRadius);
                pr.halfExtents[2] += (margin - pr.collisionRadius);
                pr.radius += (margin - pr.collisionRadius);

                pr.collisionRadius = margin;
            },
            enumerable : true
        });
    }

    Object.defineProperty(s, "halfExtents", {
        get : function shapeGetHalfExtents()
        {
            return VMath.v3Copy(this._private.halfExtents);
        },
        enumerable : true
    });

    Object.defineProperty(s, "inertia", {
        get : function shapeGetInertia()
        {
            return VMath.v3Copy(this._private.inertia);
        },
        enumerable : true
    });

    Object.defineProperty(s, "radius", {
        get : function shapeGetRadius()
        {
            return this._private.radius;
        },
        enumerable : true
    });

    Object.defineProperty(s, "type", {
        value : type,
        enumerable : true
    });
}*/
