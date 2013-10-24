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
part 'src/dullet_physics/shapes/triangle_array.dart';
part 'src/dullet_physics/shapes/triangle_mesh_shape.dart';
part 'src/dullet_physics/solver/gjk_contact_solver.dart';



class Ray {
  Ray(this.origin,this.direction,this.maxFactor);
  Vector3 origin;
  Vector3 direction;
  double maxFactor;
  //int mask;
}



class Matrix43 {
  final Float32List storage = new Float32List(12);

  Matrix43();


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
    dest.storage[9]  = -((px * m0) + (py * m1) + (pz * m2));
    dest.storage[10] = -((px * m3) + (py * m4) + (pz * m5));
    dest.storage[11] = -((px * m6) + (py * m7) + (pz * m8));
    return dest;
  }
  Matrix43 multiply(Matrix43 other, Matrix43 dest) {
    var a0  = storage[0];
    var a1  = storage[1];
    var a2  = storage[2];
    var a3  = storage[3];
    var a4  = storage[4];
    var a5  = storage[5];
    var a6  = storage[6];
    var a7  = storage[7];
    var a8  = storage[8];
    var a9  = storage[9];
    var a10 = storage[10];
    var a11 = storage[11];

    var b0  = other.storage[0];
    var b1  = other.storage[1];
    var b2  = other.storage[2];
    var b3  = other.storage[3];
    var b4  = other.storage[4];
    var b5  = other.storage[5];
    var b6  = other.storage[6];
    var b7  = other.storage[7];
    var b8  = other.storage[8];

    dest.storage[0] =  (b0 * a0 + b3 * a1 + b6 * a2);
    dest.storage[1] =  (b1 * a0 + b4 * a1 + b7 * a2);
    dest.storage[2] =  (b2 * a0 + b5 * a1 + b8 * a2);
    dest.storage[3] =  (b0 * a3 + b3 * a4 + b6 * a5);
    dest.storage[4] =  (b1 * a3 + b4 * a4 + b7 * a5);
    dest.storage[5] =  (b2 * a3 + b5 * a4 + b8 * a5);
    dest.storage[6] =  (b0 * a6 + b3 * a7 + b6 * a8);
    dest.storage[7] =  (b1 * a6 + b4 * a7 + b7 * a8);
    dest.storage[8] =  (b2 * a6 + b5 * a7 + b8 * a8);
    dest.storage[9]  = (b0 * a9 + b3 * a10 + b6 * a11 + other.storage[9]);
    dest.storage[10] = (b1 * a9 + b4 * a10 + b7 * a11 + other.storage[10]);
    dest.storage[11] = (b2 * a9 + b5 * a10 + b8 * a11 + other.storage[11]);

    return dest;
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


  double operator[](int i) => storage[i];

  void operator[]=(int i, double v) { storage[i] = v; }



}





class RayHit {
    double factor;
    Vector3 hitPoint;
    Vector3 hitNormal;
    WebGLPhysicsCollisionObject collisionObject;
    double distance;
    WebGLPhysicsPrivateBody body;
    RayHit(this.hitPoint,this.hitNormal,this.factor);
}


void calculateExtents(FakeBody fakeBody, Aabb3 extents) {
  var shape = fakeBody.shape;
  var center = shape.center;
  var halfExtents = shape.halfExtents;
  var h0 = halfExtents.storage[0];
  var h1 = halfExtents.storage[1];
  var h2 = halfExtents.storage[2];

  var transform = fakeBody.transform;
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
  var ht0 = ((m0 < 0 ? -m0 * h0 : m0 > 0 ? m0 * h0 : 0) +
      (m3 < 0 ? -m3 * h1 : m3 > 0 ? m3 * h1 : 0) +
      (m6 < 0 ? -m6 * h2 : m6 > 0 ? m6 * h2 : 0));
  var ht1 = ((m1 < 0 ? -m1 * h0 : m1 > 0 ? m1 * h0 : 0) +
      (m4 < 0 ? -m4 * h1 : m4 > 0 ? m4 * h1 : 0) +
      (m7 < 0 ? -m7 * h2 : m7 > 0 ? m7 * h2 : 0));
  var ht2 = ((m2 < 0 ? -m2 * h0 : m2 > 0 ? m2 * h0 : 0) +
      (m5 < 0 ? -m5 * h1 : m5 > 0 ? m5 * h1 : 0) +
      (m8 < 0 ? -m8 * h2 : m8 > 0 ? m8 * h2 : 0));

  extents.min.storage[0] = (ct0 - ht0);
  extents.min.storage[1] = (ct1 - ht1);
  extents.min.storage[2] = (ct2 - ht2);
  extents.max.storage[0] = (ct0 + ht0);
  extents.max.storage[1] = (ct1 + ht1);
  extents.max.storage[2] = (ct2 + ht2);
}


calculateSweptExtents(FakeBody fakeBody, Aabb3 extents) {
  var shape = fakeBody.shape;
  var radius = shape.radius;

  var startTransform = fakeBody.startTransform;
  var x0 = startTransform.storage[9];
  var x1 = startTransform.storage[10];
  var x2 = startTransform.storage[11];

  var transform = fakeBody.transform;
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
