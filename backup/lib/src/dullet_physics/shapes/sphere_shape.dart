part of dullet_physics;
//
// WebGL Physics Sphere Shape
//
class WebGLPhysicsSphereShape implements WebGLPhysicsShape
{
    //static version = 1;

    //type            : string; // prototype

    //         : WebGLPhysicsShape;
    double sphereRadius;
    double radius;
    double collisionRadius;
    Vector3 halfExtents; // v3
    Vector3 inertia; // v3
    Vector3 center; // v3
    double _margin;

    rayTest(Ray ray) {
        var origin = ray.origin;
        var direction = ray.direction;
        var radius = this.sphereRadius;

        var dir0 = direction[0];
        var dir1 = direction[1];
        var dir2 = direction[2];
        var o0 = origin[0];
        var o1 = origin[1];
        var o2 = origin[2];

        // Quadratic coeffecients at^2 + bt + c = 0
        // var a = VMath.v3Dot(direction, direction);
        // var b = 2 * VMath.v3Dot(origin, direction);
        // var c = VMath.v3Dot(origin, origin) - radius * radius;
        var a = ((dir0 * dir0) + (dir1 * dir1) + (dir2 * dir2));
        var b = (2 * ((o0 * dir0) + (o1 * dir1) + (o2 * dir2)));
        var c = (((o0 * o0) + (o1 * o1) + (o2 * o2)) - (radius * radius));

        var distance;
        // Determinant
        var d = ((b * b) - (4 * a * c));
        if (d <= 0) {
            return null;
        }

        var normalScale = 1.0;
        var rec = (1 / (2 * a));
        var rootD = Math.sqrt(d);
        distance = ((-b - rootD) * rec);
        if (distance < 0) {
            distance += (rootD * 2 * rec);
            normalScale = -1.0;
        }

        if (0 <= distance && distance < ray.maxFactor) {
            //hitPoint = VMath.v3Add(ray.origin, VMath.v3ScalarMul(ray.direction, distance));
            //hitNormal = VMath.v3ScalarDiv(hitPoint, radius * normalScale);
            var hit0 = (o0 + (dir0 * distance));
            var hit1 = (o1 + (dir1 * distance));
            var hit2 = (o2 + (dir2 * distance));

            var scale = (normalScale / radius);
            return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3((hit0 * scale), (hit1 * scale), (hit2 * scale)), distance);
        } else {
            return null;
        }
    }

    Vector3 localSupportWithoutMargin(Vector3 vec, Vector3 dst) {
      dst.setValues(0.0, 0.0, 0.0);
      return dst;
    }

    WebGLPhysicsSphereShape(var param) {
      var margin = (param.margin != null) ? param.margin : 0.04;
      var radius = param.radius;
      var i = (0.4 * radius * radius);

      sphereRadius = radius;
      radius = sphereRadius + margin;
      collisionRadius = radius + margin;
      halfExtents = new Vector3(radius + margin, radius + margin, radius + margin);
      inertia = new Vector3(i, i, i);

      center = null;
      //initShapeProperties(rets, "SPHERE", true);
      //return rets;
    }

    void set margin(double margin) {
      collisionRadius = radius + margin;
      halfExtents.x = collisionRadius;
      halfExtents.y = collisionRadius;
      halfExtents.z = collisionRadius;
      radius = collisionRadius;
    }
    double get margin => collisionRadius - radius;


}