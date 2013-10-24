part of dullet_physics;
//
// WebGL Physics Cylinder Shape
//
class WebGLPhysicsCylinderShape extends WebGLPhysicsShape
{
    //static version = 1;

    //type            : string; // prototype
    //double margin;
    //double radius;
    //Vector3 halfExtents; // v3
    //Vector3 inertia; // v3

    //_public         : WebGLPhysicsShape;
    //Vector3 center; // v3
    double cylinderRadius;
    double halfHeight;
    //double collisionRadius;

    WebGLPhysicsCylinderShape({
      double margin: 0.04,
      Vector3 halfExtents
    }) {
      //var margin = (params.margin != null) ? params.margin : 0.04;
      //var halfExtents = params.halfExtents;

      var h0 = (halfExtents[0] + margin);
      var h1 = (halfExtents[1] + margin);
      var h2 = (halfExtents[2] + margin);

      var radius2 = (h0 * h0);
      var height2 = (4.0 * h1 * h1);

      var t1 = (((1.0 / 12.0) * height2) + ((1.0 / 4.0) * radius2));
      var t2 = ((1.0 / 2.0) * radius2);

      _center = null;

      _radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
      _halfExtents = new Vector3(h0, h1, h2);
      cylinderRadius = halfExtents[0];
      halfHeight = halfExtents[1];
      _inertia = new Vector3(t1, t2, t1);
      //collisionRadius = margin;
    }




    RayHit rayTest(Ray ray) {
        var origin = ray.origin;
        var direction = ray.direction;
        var o0 = origin[0];
        var o1 = origin[1];
        var o2 = origin[2];
        var dir0 = direction[0];
        var dir1 = direction[1];
        var dir2 = direction[2];
        var maxFactor = ray.maxFactor;

        var radius = this.cylinderRadius;
        var halfHeight = this.halfHeight;
        var radius2 = radius * radius;

        // Attempt to intersect cylinder walls
        // Quadratic equation at^2 + bt + c = 0
        var a = ((dir0 * dir0) + (dir2 * dir2));
        var b = (2 * ((o0 * dir0) + (o2 * dir2)));
        var c = ((o0 * o0) + (o2 * o2) - radius2);

        var distance;
        var normalScale = 1.0;
        var hit0, hit1, hit2;
        var scale, rec;

        // Determinant
        var d = ((b * b) - (4 * a * c));
        if (d >= 0) {
          rec = (1 / (2 * a));
          var rootD = Math.sqrt(d);
          distance = ((-b - rootD) * rec);

          // don't need to check height yet. If ray's first intersection
          // is front face of cylinder, then necessarigly it is not contained
          // within the cylinder and could never hit back face first.
          if (distance < 0)
          {
              distance += (rootD * 2 * rec);
              normalScale = -1.0;
          }

          hit1 = (o1 + (dir1 * distance));
          if (-halfHeight <= hit1 && hit1 <= halfHeight) {
            if (0 <= distance && distance <= maxFactor) {
              hit0 = (o0 + (dir0 * distance));
              hit2 = (o2 + (dir2 * distance));
              scale = (normalScale / radius);
              return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3((hit0 * scale), 0.0, (hit2 * scale)), distance);
            } else {
              return null;
            }
          }
        }

        //Intersect cylinder caps
        // If ray is perpendicular to caps, we assume
        // It cannot intersect.
        if ((dir1 * dir1) >= WebGLPhysicsConfig.COPLANAR_THRESHOLD) {
          scale = ((dir1 < 0) ? -1.0 : 1.0);
          hit1 = (-scale * halfHeight);
          rec = (1 / dir1);
          distance = ((hit1 - o1) * rec);

          // Similarly don't need to check radius yet. If ray's first intersection
          // is back cap, then necesarigly it is not contained within cylinder
          // and could never hit front cap first.
          if (distance < 0) {
            hit1 = (scale * halfHeight);
            distance = ((hit1 - o1) * rec);
          }
          if (0 <= distance && distance <= maxFactor) {
            hit0 = (o0 + (dir0 * distance));
            hit2 = (o2 + (dir2 * distance));
            if (((hit0 * hit0) + (hit2 * hit2)) <= radius2) {
              return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3(0.0, -scale, 0.0), distance);
            }
          }
        }
        return null;
    }

    localSupportWithoutMargin(Vector3 vec, Vector3 dst) {
        var v0 = vec[0];
        var v2 = vec[2];
        var vmag2 = ((v0 * v0) + (v2 * v2));
        if (vmag2 == 0.0) {
          if (vec[1] > 0.0) {
            dst[0] = this.cylinderRadius;
            dst[1] = this.halfHeight;
            dst[2] = 0.0;
          } else {
            dst[0] = 0.0;
            dst[1] = -this.halfHeight;
            dst[2] = -this.cylinderRadius;
          }
          return;
        }

        var scale = (this.cylinderRadius / Math.sqrt(vmag2));
        dst[0] = (v0 * scale);
        dst[1] = ((vec[1] > 0.0 ? 1.0 : -1.0) * this.halfHeight);
        dst[2] = (v2 * scale);
    }
/*
    static create(params: any): WebGLPhysicsShape
    {
        var retc = new WebGLPhysicsShape();
        var c = new WebGLPhysicsCylinderShape();
        retc._private = c;
        c._public = retc;

        var margin = (params.margin !== undefined) ? params.margin : 0.04;
        var halfExtents = params.halfExtents;

        var h0 = (halfExtents[0] + margin);
        var h1 = (halfExtents[1] + margin);
        var h2 = (halfExtents[2] + margin);

        var radius2 = (h0 * h0);
        var height2 = (4.0 * h1 * h1);

        var t1 = (((1.0 / 12.0) * height2) + ((1.0 / 4.0) * radius2));
        var t2 = ((1.0 / 2.0) * radius2);

        c.center = undefined;

        c.radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
        c.halfExtents = VMath.v3Build(h0, h1, h2);
        c.cylinderRadius = halfExtents[0];
        c.halfHeight = halfExtents[1];
        c.inertia = VMath.v3Build(t1, t2, t1);
        c.collisionRadius = margin;

        initShapeProperties(retc, "CYLINDER");
        return retc;
    }*/
}