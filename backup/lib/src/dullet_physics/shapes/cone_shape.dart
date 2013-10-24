part of dullet_physics;
//
// WebGLPhysicsConeShape
//
class WebGLPhysicsConeShape implements WebGLPhysicsShape {

    //String type; // prototype
    double margin;
    double radius;
    Vector3 halfExtents; // v3
    Vector3 inertia; // v3

    //_public         : WebGLPhysicsShape;
    double halfHeight;
    double coneRadius;
    double collisionRadius;
    Vector3 center; // v3

    WebGLPhysicsConeShape(var param) {
      var margin = (param.margin != null) ? param.margin : 0.04;
      var radius = param.radius;
      var height = param.height;
      var halfHeight = (0.5 * height);

      var h0 = (radius + margin);
      var h1 = (halfHeight + margin);
      var h2 = (radius + margin);

      var lx = (2.0 * h0);
      var ly = (2.0 * h1);
      var lz = (2.0 * h2);
      lx *= lx;
      ly *= ly;
      lz *= lz;

      var massRatio = (1.0 / 12.0);

      halfHeight = halfHeight;
      coneRadius = radius;
      radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
      halfExtents = new Vector3(h0, h1, h2);
      inertia = new Vector3(massRatio * (ly + lz),
          massRatio * (lx + lz),
          massRatio * (lx + ly));
      collisionRadius = margin;

      center = null;
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

        var radius = this.coneRadius;
        var halfHeight = this.halfHeight;

        var conicK = (radius / (2.0 * halfHeight));
        conicK *= conicK;

        // Intersect with conic surface.
        //
        // Quadratic equation at^2 + bt + c = 0
        var d1 = o1 - halfHeight;
        var a = (dir0 * dir0) + (dir2 * dir2) - (conicK * dir1 * dir1);
        var b = 2 * ((o0 * dir0) + (o2 * dir2) - (conicK * d1 * dir1));
        var c = (o0 * o0) + (o2 * o2) - (conicK * d1 * d1);

        var distance;
        var normalScale = 1.0;
        var hit0, hit1, hit2;

        // Determinant
        var d = ((b * b) - (4 * a * c));
        if (d >= 0) {
            var rec = (1 / (2 * a));
            var rootD = Math.sqrt(d);
            distance = ((-b - rootD) * rec);
            hit1 = (o1 + (dir1 * distance));
            if (distance < 0 || hit1 < -halfHeight || hit1 > halfHeight)
            {
                distance += (2 * rootD * rec);
                normalScale = -1.0;
                hit1 = (o1 + (dir1 * distance));
                if (distance < 0 || hit1 < -halfHeight || hit1 > halfHeight)
                {
                    distance = null;
                }
            }
        }

        // Intersect with cone cap.
        var t;
        if (dir1 != 0) {
            t = (-halfHeight - o1) / dir1;
            hit0 = (o0 + (dir0 * t));
            hit2 = (o2 + (dir2 * t));
            if (t < 0 || ((hit0 * hit0) + (hit2 * hit2)) > (radius * radius))
            {
                t = null;
            }
        }

        if (t == null && distance == null) {
            return null;
        }

        if (t == null || (distance != null && distance < t)) {
            // conic surface is hit first in positive distance range
            if (distance >= maxFactor)
            {
                return null;
            }

            hit0 = (o0 + (dir0 * distance));
            hit1 = (o1 + (dir1 * distance));
            hit2 = (o2 + (dir2 * distance));

            var n1 = conicK * (hit1 - halfHeight);
            var scale = normalScale / Math.sqrt((hit0 * hit0) + (n1 * n1) + (hit2 * hit2));

            return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3(scale * hit0, scale * n1, scale * hit2), distance);
        } else {
            // cone cap is hit first in positive distance range
            if (t >= maxFactor) {
                return null;
            }

            hit0 = (o0 + (dir0 * t));
            hit1 = (o1 + (dir1 * t));
            hit2 = (o2 + (dir2 * t));
            return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3(0.0, ((o1 < -halfHeight) ? -1.0 : 1.0), 0.0), t);
        }
    }

    localSupportWithoutMargin(vec, dst) {
      var v0 = vec[0];
      var v1 = vec[1];
      var v2 = vec[2];

      var vxz = Math.sqrt((v0 * v0) + (v2 * v2));
      if (((-this.coneRadius * vxz) + (2 * this.halfHeight * v1)) > 0)
      {
          dst[0] = dst[2] = 0;
          dst[1] = this.halfHeight;
      }
      else
      {
          if (vxz == 0.0)
          {
              dst[0] = this.coneRadius;
              dst[2] = 0.0;
          }
          else
          {
              dst[0] = (v0 * this.coneRadius / vxz);
              dst[2] = (v2 * this.coneRadius / vxz);
          }
          dst[1] = -this.halfHeight;
      }
    }
/*
    static create(params: any): WebGLPhysicsShape
    {
        var retc = new WebGLPhysicsShape();
        var c = new WebGLPhysicsConeShape();
        retc._private = c;
        c._public = retc;

        var margin = (params.margin !== undefined) ? params.margin : 0.04;
        var radius = params.radius;
        var height = params.height;
        var halfHeight = (0.5 * height);

        var h0 = (radius + margin);
        var h1 = (halfHeight + margin);
        var h2 = (radius + margin);

        var lx = (2.0 * h0);
        var ly = (2.0 * h1);
        var lz = (2.0 * h2);
        lx *= lx;
        ly *= ly;
        lz *= lz;

        var massRatio = (1.0 / 12.0);

        c.halfHeight = halfHeight;
        c.coneRadius = radius;
        c.radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
        c.halfExtents = VMath.v3Build(h0, h1, h2);
        c.inertia = VMath.v3Build(massRatio * (ly + lz),
                                  massRatio * (lx + lz),
                                  massRatio * (lx + ly));
        c.collisionRadius = margin;

        c.center = undefined;

        initShapeProperties(retc, "CONE");
        return retc;
    }*/
}