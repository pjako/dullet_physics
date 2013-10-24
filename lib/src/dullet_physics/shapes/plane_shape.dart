part of dullet_physics;

// WebGLPhysicsPlaneShape.prototype.type = "PLANE";
class WebGLPhysicsPlaneShape extends WebGLPhysicsShape {
    static const int version = 1;

    // PhysicsShape
    //type            : string; // prototype
    //double margin;
    //double radius;
    //Vector3 halfExtents;    // v3
    //Vector3 inertia;    // v3

    //_public         : WebGLPhysicsShape;
    //double collisionRadius;
    double distance;
    Vector3 normal;    // v3
    //Vector3 center;    // v3



    WebGLPhysicsPlaneShape({
      double margin: 0.04,
      double distance,
      Vector3 normal
    }) {
      //var retp = new WebGLPhysicsShape();
      //var p = this;
      //retp._private = p;
      //p._public = retp;

      _collisionRadius = margin;
      this.distance = distance;
      this.normal = normal.clone();

      double maxValue = double.MAX_FINITE;

      _radius = maxValue;

      if (normal[0].abs() == 1) {
        _halfExtents = new Vector3(distance.abs(), maxValue, maxValue);
      } else if (normal[1].abs() == 1) {
        _halfExtents = new Vector3(maxValue, distance.abs(), maxValue);
      } else if (normal[2].abs() == 1) {
        _halfExtents = new Vector3(maxValue, maxValue, distance.abs());
      }

      _center = null;
      _inertia = new Vector3.zero();

      //initShapeProperties(retp, "PLANE");
      //return retp;
    }


    RayHit rayTest(Ray ray) {
        var dir = ray.direction;
        var origin = ray.origin;

        var dir0 = dir[0];
        var dir1 = dir[1];
        var dir2 = dir[2];
        var o0 = origin[0];
        var o1 = origin[1];
        var o2 = origin[2];

        var normal = this.normal;
        var n0 = normal[0];
        var n1 = normal[1];
        var n2 = normal[2];

        //var dot = VMath.v3Dot(ray.direction, this.normal);
        var dot = ((dir0 * n0) + (dir1 * n1) + (dir2 * n2));
        // If ray is parallel to plane we assume it is not
        // intersecting (Do not handle a coplanar ray)
        if ((dot * dot) < WebGLPhysicsConfig.COPLANAR_THRESHOLD) {
            return null;
        }

        //var distance = (this.distance - VMath.v3Dot(ray.origin, this.normal)) / dot;
        var distance = ((this.distance - ((o0 * n0) + (o1 * n1) + (o2 * n2))) / dot);
        if (0 <= distance && distance <= ray.maxFactor) {
            //var normal = (dot > 0) ? VMath.v3Neg(this.normal) : VMath.v3Copy(this.normal);
            if (dot > 0)
            {
                n0 = -n0;
                n1 = -n1;
                n2 = -n2;
            }
            //    hitPoint: VMath.v3Add(ray.origin, VMath.v3ScalarMul(ray.direction, distance)),
            var hit0 = (o0 + (dir0 * distance));
            var hit1 = (o1 + (dir1 * distance));
            var hit2 = (o2 + (dir2 * distance));
            return new RayHit( new Vector3(hit0, hit1, hit2), new Vector3(n0, n1, n2), distance);
        } else {
            return null;
        }
    }
}