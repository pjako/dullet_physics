part of dullet_physics;
//WebGLPhysicsShape.prototype.type = "CAPSULE";

//
// WebGL Physics Capsule Shape
//
class WebGLPhysicsCapsuleShape implements WebGLPhysicsShape {
    double radius;
    double capsuleRadius;
    double halfHeight;
    Vector3 halfExtents; // v3
    Vector3 inertia; // v3
    double collisionRadius;
    Vector3 center; // v3
    //double margin;

    rayTestCap(Ray ray, double height, double scale)
    {
      Vector3 origin = ray.origin;
      Vector3 direction = ray.direction;
      double o0 = origin[0];
      double o1 = origin[1];
      double o2 = origin[2];
      double dir0 = direction[0];
      double dir1 = direction[1];
      double dir2 = direction[2];

      double radius = this.capsuleRadius;

        //Quadratic equation at^2 + bt + c = 0
      double a = ((dir0 * dir0) + (dir1 * dir1) + (dir2 * dir2));
      double dy = (o1 - height);
      double b = (2 * ((dir0 * o0) + (dir1 * dy) + (dir2 * o2)));
      double c = ((o0 * o0) + (dy * dy) + (o2 * o2) - (radius * radius));

        //Determinant
      double d = ((b * b) - (4 * a * c));
        if (d < 0) {
            return null;
        }

        double distance;
        double normalScale = 1.0;
        double hit1;

        double rec = (1.0 / (2.0 * a));
        double rootD = Math.sqrt(d);
        distance = ((-b - rootD) * rec);
        hit1 = (o1 + (dir1 * distance));
        if (distance < 0 || (scale * (hit1 - height) < 0)) {
            distance += (2 * rootD * rec);
            hit1 = (o1 + (dir1 * distance));
            normalScale = -1.0;
        }

        if ((scale * (hit1 - height) >= 0) &&
            (0 <= distance && distance <= ray.maxFactor)) {
            var hit0 = (o0 + (dir0 * distance));
            var hit2 = (o2 + (dir2 * distance));
            var nScale = (normalScale / radius);
            return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3((hit0 * nScale), ((hit1 - height) * nScale), (hit2 * nScale)), distance);
        } else {
            return null;
        }
    }

    rayTest(Ray ray) {
      Vector3 origin = ray.origin;
      Vector3 direction = ray.direction;
        double o0 = origin[0];
        double o1 = origin[1];
        double o2 = origin[2];
        double dir0 = direction[0];
        double dir1 = direction[1];
        double dir2 = direction[2];
        double maxFactor = ray.maxFactor;

        double radius = this.capsuleRadius;
        double halfHeight = this.halfHeight;
        double radius2 = (radius * radius);

        double distance;
        double normalScale = 1.0;
        double hit0;
        double hit1;
        double hit2;

        // Attempt to intersect capsule walls
        // Quadratic equation at^2 + bt + c = 0
        var a = ((dir0 * dir0) + (dir2 * dir2));
        if (a >= WebGLPhysicsConfig.QUADRATIC_THRESHOLD)
        {
            var b = (2 * ((o0 * dir0) + (o2 * dir2)));
            var c = ((o0 * o0) + (o2 * o2) - radius2);

            // Determinant
            var d = ((b * b) - (4 * a * c));
            var rec = (1 / (2 * a));

            if (d < WebGLPhysicsConfig.QUADRATIC_THRESHOLD)
            {
                distance = (-b * rec);
            }
            else if (d > 0)
            {
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
            }

            var scale;
            hit1 = (o1 + (dir1 * distance));
            if (-halfHeight <= hit1 && hit1 <= halfHeight) {
              if (0.0 <= distance && distance <= maxFactor) {
                hit0 = (o0 + (dir0 * distance));
                hit2 = (o2 + (dir2 * distance));
                scale = (normalScale / radius);
                return new RayHit(new Vector3(hit0, hit1, hit2), new Vector3((hit0 * scale), 0.0, (hit2 * scale)), distance);
              } else {
                return null;
              }
            }
        }

        // Intersect capsule caps.
        return this.rayTestCap(ray, halfHeight, 1.0) || this.rayTestCap(ray, -halfHeight, -1.0);
    }

    localSupportWithoutMargin(Vector3 vec, Vector3 dst) {
        dst[0] = 0.0;
        dst[1] = (vec[1] >= 0.0) ? this.halfHeight : (-this.halfHeight);
        dst[2] = 0.0;
    }

    WebGLPhysicsCapsuleShape({double height, double radius, double margin: 0.04}) {


        //var margin = margin;
        //var radius = param.radius;
        //var height = param.height;
        double halfHeight = (0.5 * height);
        double maxRadius = (radius + halfHeight);

        double h0 = (radius + margin);
        double h1 = (maxRadius + margin);
        double h2 = (radius + margin);

        double lx = (2.0 * h0);
        double ly = (2.0 * h1);
        double lz = (2.0 * h2);
        lx *= lx;
        ly *= ly;
        lz *= lz;

        var massRatio = (1.0 / 12.0);

        radius = maxRadius + margin;
        capsuleRadius = radius;
        halfHeight = halfHeight;
        halfExtents = new Vector3(h0, h1, h2);
        inertia = new Vector3(massRatio * (ly + lz),
                                  massRatio * (lx + lz),
                                  massRatio * (lx + ly));
        collisionRadius = radius + margin;

        center = null;

        // Defined differently from other shapes.
        /*Object.defineProperty(retc, "margin", {
            get : function capsuleShapeGetMargin()
            {
                return (this._private.collisionRadius - this._private.capsuleRadius);
            },
            set : function capsuleShapeSetMargin(margin)
            {
                var pr = this._private;
                pr.collisionRadius = (pr.capsuleRadius + margin);
                pr.halfExtents[0] = pr.capsuleRadius + margin;
                pr.halfExtents[1] = (pr.capsuleRadius + pr.halfHeight) + margin;
                pr.halfExtents[2] = pr.capsuleRadius + margin;
                pr.radius = (pr.capsuleRadius + pr.halfHeight) + margin;
            },
            enumerable : true
        });
        initShapeProperties(retc, "CAPSULE", true);
        return retc;*/
    }

    double get margin => collisionRadius - capsuleRadius;
    void set margin(double m) {
      collisionRadius = (capsuleRadius + m);
      halfExtents[0] = capsuleRadius + m;
      halfExtents[1] = (capsuleRadius + halfHeight) + m;
      halfExtents[2] = capsuleRadius + m;
      radius = (capsuleRadius + halfHeight) + m;
    }

    capsuleShapeGetMargin() {
      return (collisionRadius - capsuleRadius);
    }
    capsuleShapeSetMargin(double margin) {
      collisionRadius = (capsuleRadius + margin);
      halfExtents[0] = capsuleRadius + margin;
      halfExtents[1] = (capsuleRadius + halfHeight) + margin;
      halfExtents[2] = capsuleRadius + margin;
      radius = (capsuleRadius + halfHeight) + margin;
    }


}