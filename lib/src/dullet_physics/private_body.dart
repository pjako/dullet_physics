part of dullet_physics;
//
// WebGLPhysicsPrivateBody
//
class WebGLPhysicsPrivateBody implements ExternalAabbTreeNode {
    static const version = 1;


    bool _fixedRotationtype = false;
    int aabbTreeIndex;
    int _id;

    //WebGLPhysicsCollisionObject _public;

    bool _previouslyActive;
    bool _bullet = false;

    // set by initPrivateBody()
    WebGLPhysicsWorld _world;
    final WebGLPhysicsShape _shape;
    double _friction = 0.5;
    double _restitution = 0.0;
    final Matrix43 _transform = new Matrix43.identity();                   // m43
    void set transform(Matrix43 t) {
      t.copyInto(_transform);
    }
    Vector3 get right => new Vector3(
        _transform.storage[0],
        _transform.storage[1],
        _transform.storage[2]);

    Vector3 get up => new Vector3(
        _transform.storage[3],
        _transform.storage[4],
        _transform.storage[5]);

    Vector3 get at => new Vector3(
        _transform.storage[6],
        _transform.storage[7],
        _transform.storage[8]);

    Vector3 get position => new Vector3(
        _transform.storage[9],
        _transform.storage[10],
        _transform.storage[11]);

    //var right = matrix.slice(0, 3);
    //var up    = matrix.slice(3, 6);
    //var at    = matrix.slice(6, 9);
    //var pos   = matrix.slice(9, 12);




    final List<WebGLPhysicsArbiter> _arbiters = new List<WebGLPhysicsArbiter>();  // TODO:
    // Tracks constraints that are inside of a space, and making use of this object.
    // We only track these constraints to avoid GC issues.
    final List _constraints = [];               // TODO:

    // [v0, v1, v2]
    // [w0, w1, w2]
    // [v0, v1, v2] <-- bias velocity
    // [w0, w1, w2] <-- bias velocity
    final Matrix43 _velocity = new Matrix43();
    double _linearDamping = 0.0;
    double _angularDamping = 0.0;
    final Aabb3 _extents = new Aabb3();

    // For continous collision detection
    final Matrix43 _startTransform = new Matrix43.identity();              // m43
    final Matrix43 _endTransform = new Matrix43.identity();                // m43
    // For kinematic objects.
    final Matrix43 _prevTransform = new Matrix43.identity();               // m43
    final Matrix43 _newTransform = new Matrix43.identity();                // m43
    WebGLPhysicsIsland _island;
    WebGLPhysicsPrivateBody _islandRoot;
    int _islandRank = 0;
    // used for kinematics so that it is kept alive for a single
    // step before being sweffed.
    bool _delaySleep = true;
    double _wakeTimeStamp;

    // set by RigidBody and CollisionObject constructors
    int _group = 0;
    int _mask = 0;
    bool _kinematic = false;
    bool _fixedRotation = false;
    double _mass = 0.0;
    double _inverseMass = 0.0;
    Vector3 _inverseInertiaLocal; // v3
    Matrix3 _inverseInertia;
    bool _collisionObject = false;
    bool _permitSleep = false;
    bool _sweepFrozen = false;
    bool _active = false;
    //contactCallbacksMask: number;
    //addedToContactCallbacks: boolean;

    WebGLPhysicsContactCallbacks _contactCallbacks;

    //TODO: Make this private
    static int uniqueId = 0;


    WebGLPhysicsPrivateBody(this._shape,
        Matrix43 transform,
        Vector3 linearVelocity,
        Vector3 angularVelocity,
        this._friction,
        this._restitution,
        this._linearDamping,
        this._angularDamping) {
        this._id = WebGLPhysicsPrivateBody.uniqueId;
        WebGLPhysicsPrivateBody.uniqueId += 1;
        if(transform != null) {
          transform.copyInto(_transform);
        }
        var vel = linearVelocity;
        if (vel != null) {
            _velocity.storage[0] = vel.storage[0];
            _velocity.storage[1] = vel.storage[1];
            _velocity.storage[2] = vel.storage[2];
        }
        vel = angularVelocity;
        if (vel != null) {
            _velocity.storage[3] = vel.storage[0];
            _velocity.storage[4] = vel.storage[1];
            _velocity.storage[5] = vel.storage[2];
        }
        this._islandRoot = this;
    }

    // Used for kinematics.
    // TODO: Should be used for convexSweep to permit non-linear sweeps.
    bool _computeDeltaVelocity(double timeStep, Matrix43 from, Matrix43 to, Matrix43 inputVelocity) {
      // was:
      // var velocity = inputVelocity || this.velocity;
      var velocity = inputVelocity;
      if(velocity == null) {
        velocity = this._velocity;
      }

      var active = false;
      double from0 = from[0];
      double from1 = from[1];
      double from2 = from[2];
      double from3 = from[3];
      double from4 = from[4];
      double from5 = from[5];
      double from6 = from[6];
      double from7 = from[7];
      double from8 = from[8];
      double from9 = from[9];
      double from10 = from[10];
      double from11 = from[11];

      double to0 = to[0];
      double to1 = to[1];
      double to2 = to[2];
      double to3 = to[3];
      double to4 = to[4];
      double to5 = to[5];
      double to6 = to[6];
      double to7 = to[7];
      double to8 = to[8];
      double to9 = to[9];
      double to10 = to[10];
      double to11 = to[11];

      double velocityX = (to9  - from9);
      double velocityY = (to10 - from10);
      double velocityZ = (to11 - from11);
      if (velocityX != 0.0 ||
          velocityY != 0.0 ||
          velocityZ != 0.0)
      {
          active = true;
      }

      // do this afterwards so that active is true, even if timeStep is 0
      // for non-equal position transforms.
      velocity[0] = velocityX / timeStep;
      velocity[1] = velocityY / timeStep;
      velocity[2] = velocityZ / timeStep;



      //var deltaRotation = VMath.m33Mul(VMath.m33Inverse(from), to);
      var m0 = (from0 * to0) + (from3 * to3) + (from6 * to6);
      var m1 = (from0 * to1) + (from3 * to4) + (from6 * to7);
      var m2 = (from0 * to2) + (from3 * to5) + (from6 * to8);
      var m3 = (from1 * to0) + (from4 * to3) + (from7 * to6);
      var m4 = (from1 * to1) + (from4 * to4) + (from7 * to7);
      var m5 = (from1 * to2) + (from4 * to5) + (from7 * to8);
      var m6 = (from2 * to0) + (from5 * to3) + (from8 * to6);
      var m7 = (from2 * to1) + (from5 * to4) + (from8 * to7);
      var m8 = (from2 * to2) + (from5 * to5) + (from8 * to8);

      //var quat = VMath.quatFromM43(deltaRotation);
      double x, y, z, w, s;
      var trace = m0 + m4 + m8 + 1;
      if (trace > MathHelper.precision) {
          w = Math.sqrt(trace) / 2;
          x = (m5 - m7) / (4 * w);
          y = (m6 - m2) / (4 * w);
          z = (m1 - m3) / (4 * w);
      } else {
          if ((m0 > m4) && (m0 > m8)) {
              s = Math.sqrt(1.0 + m0 - m4 - m8) * 2.0; // S=4*qx
              w = (m5 - m7) / s;
              x = 0.25 * s;
              y = (m3 + m1) / s;
              z = (m6 + m2) / s;
          } else if (m4 > m8) {
              s = Math.sqrt(1.0 + m4 - m0 - m8) * 2.0; // S=4*qy
              w = (m6 - m2) / s;
              x = (m3 + m1) / s;
              y = 0.25 * s;
              z = (m7 + m5) / s;
          } else {
              s = Math.sqrt(1.0 + m8 - m0 - m4) * 2.0; // S=4*qz
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
    void _calculateSweptExtents(Aabb3 extents) {
        var _shape = this._shape;
        var radius = _shape._radius;

        var startTransform = this._startTransform;
        var x0 = startTransform.storage[9];
        var x1 = startTransform.storage[10];
        var x2 = startTransform.storage[11];

        var transform = this._transform;
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
    void _calculateExtents(Aabb3 extents) {
        var shape = this._shape;
        var center = shape._center;
        var halfExtents = shape._halfExtents.storage;
        var h0 = halfExtents[0];
        var h1 = halfExtents[1];
        var h2 = halfExtents[2];

        var transform = this._transform.storage;
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
        if (center != null)
        {
            var c0 = center.storage[0];
            var c1 = center.storage[1];
            var c2 = center.storage[2];

            if (c0 != 0.0 ||
                c1 != 0.0 ||
                c2 != 0.0)
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

    // used for all types.
    RayHit _rayTest(Ray ray) {
      //Transform ray; assuming transform is orthogonal
      var transform = this._transform;
      var rayT = new Ray(
          transform.inverseOrthonormalTransformPoint(ray.origin, new Vector3.zero()),
          transform.inverseOrthonormalTransformVector(ray.direction, new Vector3.zero()),
          ray.maxFactor
      );

      var result = this._shape.rayTest(rayT);
      if (result != null) {
          result.hitPoint = MathHelper.m43TransformPoint(transform, result.hitPoint, result.hitPoint);
          result.hitNormal = MathHelper.m43TransformVector(transform, result.hitNormal, result.hitNormal);
      }

      return result;
    }

    // used for kinematics and dynamics
    void _integratePositionWithVelocities(Matrix43 transform, Matrix43 outTransform, double timeStep, int offset){
      var velocity = this._velocity;

      // x += h * v
      outTransform.storage[9]  = transform.storage[9]  + (timeStep * velocity.storage[offset]);
      outTransform.storage[10] = transform.storage[10] + (timeStep * velocity.storage[offset + 1]);
      outTransform.storage[11] = transform.storage[11] + (timeStep * velocity.storage[offset + 2]);

      // A += h * skew(w) * A
      var w0 = velocity.storage[offset + 3] * timeStep;
      var w1 = velocity.storage[offset + 4] * timeStep;
      var w2 = velocity.storage[offset + 5] * timeStep;

      var A0 = transform.storage[0];
      var A1 = transform.storage[1];
      var A2 = transform.storage[2];
      var A3 = transform.storage[3];
      var A4 = transform.storage[4];
      var A5 = transform.storage[5];
      var A6 = transform.storage[6];
      var A7 = transform.storage[7];
      var A8 = transform.storage[8];

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
      var scale = 1.0 / Math.sqrt((B0 * B0) + (B1 * B1) + (B2 * B2));
      B0 *= scale;
      B1 *= scale;
      B2 *= scale;

      scale = -((B0 * B3) + (B1 * B4) + (B2 * B5));
      B3 += B0 * scale;
      B4 += B1 * scale;
      B5 += B2 * scale;

      scale = 1.0 / Math.sqrt((B3 * B3) + (B4 * B4) + (B5 * B5));
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

      scale = 1.0 / Math.sqrt((B6 * B6) + (B7 * B7) + (B8 * B8));
      B6 *= scale;
      B7 *= scale;
      B8 *= scale;

      outTransform.storage[0] = B0;
      outTransform.storage[1] = B1;
      outTransform.storage[2] = B2;
      outTransform.storage[3] = B3;
      outTransform.storage[4] = B4;
      outTransform.storage[5] = B5;
      outTransform.storage[6] = B6;
      outTransform.storage[7] = B7;
      outTransform.storage[8] = B8;
    }

    // used for dynamics.
    void _applyBiasVelocities(double timeStep){
        var velocity = this._velocity;
        this._integratePositionWithVelocities(this._transform, this._startTransform, timeStep, 6);

        // Set bias velocities back to 0.
        velocity.storage[6] = velocity.storage[7] = velocity.storage[8] = 0.0;
        velocity.storage[9] = velocity.storage[10] = velocity.storage[11] = 0.0;
    }

    // used for kinematics and dynamics.
    void _integratePosition(double timeStep) {
        this._integratePositionWithVelocities(this._startTransform, this._transform, timeStep, 0);
    }


    void calculateTransform(Matrix43 transform, [Vector3 origin]) {
      var privateTransform = this._transform;
      if (origin != null) {
        privateTransform.negOffset(origin, transform);
      } else {
        privateTransform.copyInto(transform);
      }
    }

    // used for dynamics.
    void _refreshInertiaTensor() {
        var A = this._transform.storage;
        var inertia = this._inverseInertiaLocal.storage;
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
        var I = this._inverseInertia.storage;
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
    void _integrateVelocity(Vector3 gravity, double timeStep) {
        var velocity = this._velocity;

        var pow = Math.pow;
        // v += h * F / m. Damping applied directly.
        var linDrag = pow(1.0 - this._linearDamping, timeStep);
        velocity.storage[0] = (velocity.storage[0] + (timeStep * gravity.storage[0])) * linDrag;
        velocity.storage[1] = (velocity.storage[1] + (timeStep * gravity.storage[1])) * linDrag;
        velocity.storage[2] = (velocity.storage[2] + (timeStep * gravity.storage[2])) * linDrag;

        var angDrag = pow(1 - this._angularDamping, timeStep);
        var w0 = velocity.storage[3] * angDrag;
        var w1 = velocity.storage[4] * angDrag;
        var w2 = velocity.storage[5] * angDrag;

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
    bool _isActiveVelocity(double linear, double angular) {
        var r = _shape._radius;

        var velocity = this._velocity;
        var v0 = velocity.storage[0];
        var v1 = velocity.storage[1];
        var v2 = velocity.storage[2];
        var vmag = ((v0 * v0) + (v1 * v1) + (v2 * v2));
        if (vmag > (linear * r * r))
        {
            return true;
        }

        v0 = velocity.storage[3];
        v1 = velocity.storage[4];
        v2 = velocity.storage[5];
        if (((v0 * v0) + (v1 * v1) + (v2 * v2)) > angular)
        {
            return true;
        }

        return false;
    }

    // Return false if body is (taking into account sleep delay) able to sleep.
    // used for dynamics.
    bool isActive(/* timeStep */) {
        if (!this._permitSleep) {
          return true;
        }

        if (this._isActiveVelocity(WebGLPhysicsConfig.SLEEP_LINEAR_SQ, WebGLPhysicsConfig.SLEEP_ANGULAR_SQ)) {
          this._wakeTimeStamp = this._world._timeStamp;
          return true;
        }
        return ((this._wakeTimeStamp + WebGLPhysicsConfig.SLEEP_DELAY) > this._world._timeStamp);
    }
}



/*
 * from the constructor
 *
this._world = null;
this._shape = shape;

this._friction    = friction;
this._restitution = restitution;
_linearDamping = linearDamping;
_angularDamping = angularDamping;



//this.linearDamping  = (params.linearDamping  != null) ? params.linearDamping  : 0.0;
//this.angularDamping = (params.angularDamping != null) ? params.angularDamping : 0.0;

//this._extents = new Aabb3();


this._island = null;
this._islandRank = 0;
this._delaySleep = true;
this._group = 0;
this._mask = 0;
this._kinematic = false;
this._fixedRotation = false;
this._mass = 0.0;
this._inverseMass = 0.0;
this._inverseInertiaLocal = null;
this._inverseInertia = null;
this._collisionObject = false;
this._permitSleep = false;
this._sweepFrozen = false;
this._active = false;
this._contactCallbacks = null;
*/