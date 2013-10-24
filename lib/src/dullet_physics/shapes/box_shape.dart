part of dullet_physics;
//
// WebGL Physics Box Shape
//
class WebGLPhysicsBoxShape extends WebGLPhysicsShape {
    //double margin;
    //double radius;
    //Vector3 halfExtents; // v3
    //Vector3 inertia; // v3
    //Vector3 center; // v3
    //double collisionRadius;

    WebGLPhysicsBoxShape(Vector3 halfExtents, {double margin: 0.04}) {
      //double margin = margin;
      //Vector3 halfExtents = halfExtents;

      var h0 = (halfExtents[0] + margin);
      var h1 = (halfExtents[1] + margin);
      var h2 = (halfExtents[2] + margin);

      var lx = (2.0 * h0);
      var ly = (2.0 * h1);
      var lz = (2.0 * h2);
      lx *= lx;
      ly *= ly;
      lz *= lz;

      _center = null;

      _radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
      this._halfExtents = new Vector3(h0, h1, h2);
      _inertia = new Vector3(
          (1.0 / 12.0) * (ly + lz),
          (1.0 / 12.0) * (lx + lz),
          (1.0 / 12.0) * (lx + ly));
      _collisionRadius = margin;
    }

    RayHit rayTest(Ray ray) {
      var origin = ray.origin;
      var direction = ray.direction;
      var o0 = origin[0];
      var o1 = origin[1];
      var o2 = origin[2];
      var d0 = direction[0];
      var d1 = direction[1];
      var d2 = direction[2];

      var halfExtents = this._halfExtents;
      var h0 = halfExtents[0];
      var h1 = halfExtents[1];
      var h2 = halfExtents[2];

      var minDistance;
      var axis;

      // Code is similar for all pairs of faces.
      // Could be moved to a function, but would have performance penalty.
      //
      // In each case we check (Assuming that ray is not horizontal to plane)
      // That the ray is able to intersect one or both of the faces' planes
      // based on direction, origin and half extents.
      //                        |    |
      // cannot intersect <--o  |    | o--> cannot intersect
      //                        |    |
      //
      // If ray is able to intersect planes, we choose which face to intersect
      // with based on direction, origin and half extents and perform intersection.
      //                           |           |
      //                           | o--> pos. | <--o intersect pos. face
      // intersect neg. face o-->  | neg. <--o |
      //                           |           |
      //

      // intersect with yz faces.
      var t, f, hx, hy;
      if (d0 != 0 && ((d0 > 0 && o0 <= -h0) || (d0 < 0 && o0 >= h0)))
      {
          f = (d0 > 0 ? (o0 >= -h0 ? h0 : -h0) : (o0 <= h0 ? -h0 : h0));
          t = (f - o0) / d0;
          if (minDistance == null || t < minDistance)
          {
              hx = o1 + (d1 * t);
              hy = o2 + (d2 * t);
              if ((-h1 <= hx && hx <= h1) &&
                  (-h2 <= hy && hy <= h2))
              {
                  minDistance = t;
                  axis = 0;
              }
          }
      }

      // intersect with xz faces.
      if (d1 != 0 && ((d1 > 0 && o1 <= -h1) || (d1 < 0 && o1 >= h1)))
      {
          f = (d1 > 0 ? (o1 >= -h1 ? h1 : -h1) : (o1 <= h1 ? -h1 : h1));
          t = (f - o1) / d1;
          if (minDistance == null || t < minDistance)
          {
              hx = o0 + (d0 * t);
              hy = o2 + (d2 * t);
              if ((-h0 <= hx && hx <= h0) &&
                  (-h2 <= hy && hy <= h2))
              {
                  minDistance = t;
                  axis = 1;
              }
          }
      }

      // intersect with xy faces.
      if (d2 != 0 && ((d2 > 0 && o2 <= -h2) || (d2 < 0 && o2 >= h2)))
      {
          f = (d2 > 0 ? (o2 >= -h2 ? h2 : -h2) : (o2 <= h2 ? -h2 : h2));
          t = (f - o2) / d2;
          if (minDistance == null || t < minDistance)
          {
              hx = o1 + (d1 * t);
              hy = o0 + (d0 * t);
              if ((-h1 <= hx && hx <= h1) &&
                  (-h0 <= hy && hy <= h0))
              {
                  minDistance = t;
                  axis = 2;
              }
          }
      }

      if (minDistance != null && minDistance < ray.maxFactor) {
          return new RayHit(
              new Vector3(o0 + d0 * minDistance,
                            o1 + d1 * minDistance,
                            o2 + d2 * minDistance),
              new Vector3(axis == 0 ? (d0 > 0 ? -1 : 1.0) : 0.0,
                            axis == 1 ? (d1 > 0 ? -1 : 1.0) : 0.0,
                            axis == 2 ? (d2 > 0 ? -1 : 1.0) : 0.0),
              minDistance);

      } else {
          return null;
      }
    }

    void localSupportWithoutMargin(Vector3 vec, Vector3 dst) {
        var v0 = vec.storage[0];
        var v1 = vec.storage[1];
        var v2 = vec.storage[2];

        var halfExtents = this._halfExtents;
        var h0 = halfExtents.storage[0];
        var h1 = halfExtents.storage[1];
        var h2 = halfExtents.storage[2];

        dst.storage[0] = ((v0 < 0.0) ? -h0 : h0);
        dst.storage[1] = ((v1 < 0.0) ? -h1 : h1);
        dst.storage[2] = ((v2 < 0.0) ? -h2 : h2);
    }



/*
    static create(params): WebGLPhysicsShape
    {
        var retb = new WebGLPhysicsShape();
        var b = new WebGLPhysicsBoxShape();
        retb._private = b;
        b._public = retb;

        var margin = (params.margin !== undefined) ? params.margin : 0.04;
        var halfExtents = params.halfExtents;

        var h0 = (halfExtents[0] + margin);
        var h1 = (halfExtents[1] + margin);
        var h2 = (halfExtents[2] + margin);

        var lx = (2.0 * h0);
        var ly = (2.0 * h1);
        var lz = (2.0 * h2);
        lx *= lx;
        ly *= ly;
        lz *= lz;

        b.center = undefined;

        b.radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
        b.halfExtents = VMath.v3Build(h0, h1, h2);
        b.inertia = VMath.v3Build((1.0 / 12.0) * (ly + lz),
                                  (1.0 / 12.0) * (lx + lz),
                                  (1.0 / 12.0) * (lx + ly));
        b.collisionRadius = margin;

        initShapeProperties(retb, "BOX");
        return retb;
    }*/
}