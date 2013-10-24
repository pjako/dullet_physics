part of dullet_physics;
//
// WebGLPhysicsPrivateBody
//
class WebGLPhysicsPrivateBody implements ExternalAabbTreeNode {
    static const version = 1;


    bool fixedRotationtype;
    int aabbTreeIndex;
    int id;

    WebGLPhysicsCollisionObject _public;

    bool previouslyActive;
    bool bullet;

    // set by initPrivateBody()
    WebGLPrivatePhysicsWorld world;
    WebGLPhysicsShape shape;
    double friction;
    double restitution;
    Matrix43 transform;                   // m43
    List<WebGLPhysicsArbiter> arbiters;  // TODO:
    List constraints;               // TODO:
    Float32List velocity;           // length 12
    double linearDamping;
    double angularDamping;
    Float32List extents;            // length 6
    Matrix43 startTransform;              // m43
    Matrix43 endTransform;                // m43
    Matrix43 prevTransform;               // m43
    Matrix43 newTransform;                // m43
    WebGLPhysicsIsland island;
    WebGLPhysicsPrivateBody islandRoot;
    int islandRank;
    bool delaySleep;
    double wakeTimeStamp;

    // set by RigidBody and CollisionObject constructors
    int group;
    int mask;
    bool kinematic;
    bool fixedRotation;
    double mass;
    double inverseMass;
    Vector3 inverseInertiaLocal; // v3
    Float32List inverseInertia;
    bool collisionObject;
    bool permitSleep;
    bool sweepFrozen;
    bool active;
    //contactCallbacksMask: number;
    //addedToContactCallbacks: boolean;

    WebGLPhysicsContactCallbacks contactCallbacks;

    //TODO: Make this private
    static int uniqueId = 0;

    WebGLPhysicsPrivateBody({
      WebGLPhysicsCollisionObject publicObject,
      WebGLPhysicsShape shape,
      Float32List transform,
      Float32List linearVelocity,
      Float32List angularVelocity,
      double friction: 0.5,
      double restitution: 0.0,
      double linearDamping: 0.0,
      double angularDamping: 0.0
      }) {
        this._public = publicObject;

        this.id = WebGLPhysicsPrivateBody.uniqueId;
        WebGLPhysicsPrivateBody.uniqueId += 1;

        this.world = null;
        this.shape = shape._private;

        this.friction    = friction;
        this.restitution = restitution;

        var xform = transform;
        this.transform = (xform ? MathHelper.m43Copy(xform) : MathHelper.m43BuildIdentity());

        this.arbiters = [];
        // Tracks constraints that are inside of a space, and making use of this object.
        // We only track these constraints to avoid GC issues.
        this.constraints = [];

        // [v0, v1, v2]
        // [w0, w1, w2]
        // [v0, v1, v2] <-- bias velocity
        // [w0, w1, w2] <-- bias velocity
        this.velocity = new Float32List(12);
        var vel = linearVelocity;
        if (vel != null) {
            this.velocity[0] = vel[0];
            this.velocity[1] = vel[1];
            this.velocity[2] = vel[2];
        }
        vel = angularVelocity;
        if (vel != null) {
            this.velocity[3] = vel[0];
            this.velocity[4] = vel[1];
            this.velocity[5] = vel[2];
        }

        //this.linearDamping  = (params.linearDamping  != null) ? params.linearDamping  : 0.0;
        //this.angularDamping = (params.angularDamping != null) ? params.angularDamping : 0.0;

        this.extents = new Float32List(6);

        // For continous collision detection
        this.startTransform = MathHelper.m43BuildIdentity();
        this.endTransform = MathHelper.m43BuildIdentity();

        // For kinematic objects.
        this.prevTransform = MathHelper.m43Copy(this.transform);
        this.newTransform = MathHelper.m43BuildIdentity();

        this.island = null;
        this.islandRoot = this;
        this.islandRank = 0;

        // used for kinematics so that it is kept alive for a single
        // step before being sweffed.
        this.delaySleep = true;

        this.group = 0;
        this.mask = 0;
        this.kinematic = false;
        this.fixedRotation = false;
        this.mass = 0.0;
        this.inverseMass = 0.0;
        this.inverseInertiaLocal = null;
        this.inverseInertia = null;
        this.collisionObject = false;
        this.permitSleep = false;
        this.sweepFrozen = false;
        this.active = false;
        this.contactCallbacks = null;
    }

    // Used for kinematics.
    // TODO: Should be used for convexSweep to permit non-linear sweeps.
    bool computeDeltaVelocity(double timeStep, Vector3 from, Vector3 to, Float32List inputVelocity) {
      // was:
      // var velocity = inputVelocity || this.velocity;
      var velocity = inputVelocity;
      if(velocity == null) {
        velocity = this.velocity;
      }

      var active = false;

      velocity[0] = (to[9]  - from[9]);
      velocity[1] = (to[10] - from[10]);
      velocity[2] = (to[11] - from[11]);
      if (velocity[0] != 0 ||
          velocity[1] != 0 ||
          velocity[2] != 0)
      {
          active = true;
      }

      // do this afterwards so that active is true, even if timeStep is 0
      // for non-equal position transforms.
      velocity[0] /= timeStep;
      velocity[1] /= timeStep;
      velocity[2] /= timeStep;

      //var deltaRotation = VMath.m33Mul(VMath.m33Inverse(from), to);
      var m0 = (from[0] * to[0]) + (from[3] * to[3]) + (from[6] * to[6]);
      var m1 = (from[0] * to[1]) + (from[3] * to[4]) + (from[6] * to[7]);
      var m2 = (from[0] * to[2]) + (from[3] * to[5]) + (from[6] * to[8]);
      var m3 = (from[1] * to[0]) + (from[4] * to[3]) + (from[7] * to[6]);
      var m4 = (from[1] * to[1]) + (from[4] * to[4]) + (from[7] * to[7]);
      var m5 = (from[1] * to[2]) + (from[4] * to[5]) + (from[7] * to[8]);
      var m6 = (from[2] * to[0]) + (from[5] * to[3]) + (from[8] * to[6]);
      var m7 = (from[2] * to[1]) + (from[5] * to[4]) + (from[8] * to[7]);
      var m8 = (from[2] * to[2]) + (from[5] * to[5]) + (from[8] * to[8]);

      //var quat = VMath.quatFromM43(deltaRotation);
      var x, y, z, w, s;
      var trace = m0 + m4 + m8 + 1;
      if (trace > MathHelper.precision) {
          w = Math.sqrt(trace) / 2;
          x = (m5 - m7) / (4 * w);
          y = (m6 - m2) / (4 * w);
          z = (m1 - m3) / (4 * w);
      } else {
          if ((m0 > m4) && (m0 > m8)) {
              s = Math.sqrt(1.0 + m0 - m4 - m8) * 2; // S=4*qx
              w = (m5 - m7) / s;
              x = 0.25 * s;
              y = (m3 + m1) / s;
              z = (m6 + m2) / s;
          } else if (m4 > m8) {
              s = Math.sqrt(1.0 + m4 - m0 - m8) * 2; // S=4*qy
              w = (m6 - m2) / s;
              x = (m3 + m1) / s;
              y = 0.25 * s;
              z = (m7 + m5) / s;
          } else {
              s = Math.sqrt(1.0 + m8 - m0 - m4) * 2; // S=4*qz
              w = (m1 - m3) / s;
              x = (m6 + m2) / s;
              y = (m7 + m5) / s;
              z = 0.25 * s;
          }
      }

      var angle = Math.acos(w) * 2.0;
      var sin_sqrd = 1.0 - (w * w);
      if (sin_sqrd < MathHelper.precision || angle == 0.0)
      {
          velocity[3] = velocity[4] = velocity[5] = 0.0;
      } else {
          var scale = angle / (timeStep * Math.sqrt(sin_sqrd));
          velocity[3] = x * scale;
          velocity[4] = y * scale;
          velocity[5] = z * scale;
          active = true;
      }

      return active;
    }

    // Used for kinematic and dynamics.
    calculateSweptExtents(Aabb3 extents) {
        var shape = this.shape;
        var radius = shape.radius;

        var startTransform = this.startTransform;
        var x0 = startTransform.storage[9];
        var x1 = startTransform.storage[10];
        var x2 = startTransform.storage[11];

        var transform = this.transform;
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

    // use for all types.
    void calculateExtents(Float32List extents) {
        var shape = this.shape;
        var center = shape.center;
        var halfExtents = shape.halfExtents;
        var h0 = halfExtents[0];
        var h1 = halfExtents[1];
        var h2 = halfExtents[2];

        var transform = this.transform;
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

        extents[0] = (ct0 - ht0);
        extents[1] = (ct1 - ht1);
        extents[2] = (ct2 - ht2);
        extents[3] = (ct0 + ht0);
        extents[4] = (ct1 + ht1);
        extents[5] = (ct2 + ht2);
    }

    // used for all types.
    RayHit rayTest(Ray ray) {
      //Transform ray; assuming transform is orthogonal
      var transform = this.transform;
      var rayT = new Ray(
          transform.inverseOrthonormalTransformPoint(ray.origin, new Vector3.zero()),
          transform.inverseOrthonormalTransformVector(ray.direction, new Vector3.zero()),
          ray.maxFactor
      );

      var result = this.shape.rayTest(rayT);
      if (result != null) {
          result.hitPoint = MathHelper.m43TransformPoint(transform, result.hitPoint, result.hitPoint);
          result.hitNormal = MathHelper.m43TransformVector(transform, result.hitNormal, result.hitNormal);
      }

      return result;
    }

    // used for kinematics and dynamics
    void integratePositionWithVelocities(Float32List transform, Float32List outTransform, double timeStep, int offset){
      var velocity = this.velocity;
      var sqrt = Math.sqrt;

      // x += h * v
      outTransform[9]  = transform[9]  + (timeStep * velocity[offset]);
      outTransform[10] = transform[10] + (timeStep * velocity[offset + 1]);
      outTransform[11] = transform[11] + (timeStep * velocity[offset + 2]);

      // A += h * skew(w) * A
      var w0 = velocity[offset + 3] * timeStep;
      var w1 = velocity[offset + 4] * timeStep;
      var w2 = velocity[offset + 5] * timeStep;

      var A0 = transform[0];
      var A1 = transform[1];
      var A2 = transform[2];
      var A3 = transform[3];
      var A4 = transform[4];
      var A5 = transform[5];
      var A6 = transform[6];
      var A7 = transform[7];
      var A8 = transform[8];

      var B0 = A0 - (w2 * A1) + (w1 * A2);
      var B1 = A1 + (w2 * A0) - (w0 * A2);
      var B2 = A2 - (w1 * A0) + (w0 * A1);
      var B3 = A3 - (w2 * A4) + (w1 * A5);
      var B4 = A4 + (w2 * A3) - (w0 * A5);
      var B5 = A5 - (w1 * A3) + (w0 * A4);
      var B6 = A6 - (w2 * A7) + (w1 * A8);
      var B7 = A7 + (w2 * A6) - (w0 * A8);
      var B8 = A8 - (w1 * A6) + (w0 * A7);

      // Orthornormalize with modified gram schmidt.
      var scale = 1 / sqrt((B0 * B0) + (B1 * B1) + (B2 * B2));
      B0 *= scale;
      B1 *= scale;
      B2 *= scale;

      scale = -((B0 * B3) + (B1 * B4) + (B2 * B5));
      B3 += B0 * scale;
      B4 += B1 * scale;
      B5 += B2 * scale;

      scale = 1 / sqrt((B3 * B3) + (B4 * B4) + (B5 * B5));
      B3 *= scale;
      B4 *= scale;
      B5 *= scale;

      scale = -((B0 * B6) + (B1 * B7) + (B2 * B8));
      B6 += B0 * scale;
      B7 += B1 * scale;
      B8 += B2 * scale;

      scale = -((B3 * B6) + (B4 * B7) + (B5 * B8));
      B6 += B3 * scale;
      B7 += B4 * scale;
      B8 += B5 * scale;

      scale = 1 / sqrt((B6 * B6) + (B7 * B7) + (B8 * B8));
      B6 *= scale;
      B7 *= scale;
      B8 *= scale;

      outTransform[0] = B0;
      outTransform[1] = B1;
      outTransform[2] = B2;
      outTransform[3] = B3;
      outTransform[4] = B4;
      outTransform[5] = B5;
      outTransform[6] = B6;
      outTransform[7] = B7;
      outTransform[8] = B8;
    }

    // used for dynamics.
    void applyBiasVelocities(double timeStep){
        var velocity = this.velocity;
        this.integratePositionWithVelocities(this.transform, this.startTransform, timeStep, 6);

        // Set bias velocities back to 0.
        velocity[6] = velocity[7] = velocity[8] = 0;
        velocity[9] = velocity[10] = velocity[11] = 0;
    }

    // used for kinematics and dynamics.
    void integratePosition(double timeStep) {
        this.integratePositionWithVelocities(this.startTransform, this.transform, timeStep, 0);
    }

    // used for dynamics.
    void refreshInertiaTensor() {
        var A = this.transform;
        var inertia = this.inverseInertiaLocal;
        var i0 = inertia[0];
        var i1 = inertia[1];
        var i2 = inertia[2];

        var A0 = A[0];
        var A1 = A[1];
        var A2 = A[2];
        var A3 = A[3];
        var A4 = A[4];
        var A5 = A[5];
        var A6 = A[6];
        var A7 = A[7];
        var A8 = A[8];

        // I = A * 1/I' * A^T
        var I = this.inverseInertia;
        I[0] = (A0 * A0 * i0) + (A3 * A3 * i1) + (A6 * A6 * i2);
        I[1] = (A0 * A1 * i0) + (A3 * A4 * i1) + (A6 * A7 * i2);
        I[2] = (A0 * A2 * i0) + (A3 * A5 * i1) + (A6 * A8 * i2);
        I[3] = (A1 * A0 * i0) + (A4 * A3 * i1) + (A7 * A6 * i2);
        I[4] = (A1 * A1 * i0) + (A4 * A4 * i1) + (A7 * A7 * i2);
        I[5] = (A1 * A2 * i0) + (A4 * A5 * i1) + (A7 * A8 * i2);
        I[6] = (A2 * A0 * i0) + (A5 * A3 * i1) + (A8 * A6 * i2);
        I[7] = (A2 * A1 * i0) + (A5 * A4 * i1) + (A8 * A7 * i2);
        I[8] = (A2 * A2 * i0) + (A5 * A5 * i1) + (A8 * A8 * i2);
    }

    // used for dynamics.
    void integrateVelocity(Float32List gravity, double timeStep) {
        var velocity = this.velocity;

        var pow = Math.pow;
        // v += h * F / m. Damping applied directly.
        var linDrag = pow(1 - this.linearDamping, timeStep);
        velocity[0] = (velocity[0] + (timeStep * gravity[0])) * linDrag;
        velocity[1] = (velocity[1] + (timeStep * gravity[1])) * linDrag;
        velocity[2] = (velocity[2] + (timeStep * gravity[2])) * linDrag;

        var angDrag = pow(1 - this.angularDamping, timeStep);
        var w0 = velocity[3] * angDrag;
        var w1 = velocity[4] * angDrag;
        var w2 = velocity[5] * angDrag;

        // Apply clamping to angularVelocity
        var max_angular = WebGLPhysicsConfig.MAX_ANGULAR / timeStep;
        var wlsq = ((w0 * w0) + (w1 * w1) + (w2 * w2));
        if (wlsq > (max_angular * max_angular))
        {
            var scale = max_angular / Math.sqrt(wlsq);
            w0 *= scale;
            w1 *= scale;
            w2 *= scale;
        }

        velocity[3] = w0;
        velocity[4] = w1;
        velocity[5] = w2;
    }

    // Return false if body is (considering purely velocity) able to sleep.
    // used for dynamics.
    bool isActiveVelocity(double linear, double angular) {
        var r = this.shape.radius;

        var velocity = this.velocity;
        var v0 = velocity[0];
        var v1 = velocity[1];
        var v2 = velocity[2];
        var vmag = ((v0 * v0) + (v1 * v1) + (v2 * v2));
        if (vmag > (linear * r * r))
        {
            return true;
        }

        v0 = velocity[3];
        v1 = velocity[4];
        v2 = velocity[5];
        if (((v0 * v0) + (v1 * v1) + (v2 * v2)) > angular)
        {
            return true;
        }

        return false;
    }

    // Return false if body is (taking into account sleep delay) able to sleep.
    // used for dynamics.
    bool isActive(/* timeStep */) {
        if (!this.permitSleep) {
          return true;
        }

        if (this.isActiveVelocity(WebGLPhysicsConfig.SLEEP_LINEAR_SQ, WebGLPhysicsConfig.SLEEP_ANGULAR_SQ)) {
          this.wakeTimeStamp = this.world.timeStamp;
          return true;
        }
        return ((this.wakeTimeStamp + WebGLPhysicsConfig.SLEEP_DELAY) > this.world.timeStamp);
    }
}