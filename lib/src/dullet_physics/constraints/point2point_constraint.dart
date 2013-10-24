part of dullet_physics;


//
// WebGLPhysicsPoint2Point Constraint
//
class WebGLPhysicsPoint2PointConstraint extends WebGLPhysicsConstraint /*implements PhysicsPoint2PointConstraint*/ {
    static const int version = 1;
    //String type = "POINT2POINT";  // prototype

    // PhysicsConstraint
    //WebGLPhysicsCollisionObject bodyA;
    //WebGLPhysicsCollisionObject bodyB;
    //Matrix43 transformA;//: any; // m43
    //Matrix43 transformB;//: any; // m43
    // TODO: move into the class?
    //WebGLPhysicsPrivatePoint2PointConstraint _private;
    Float32List _data = new Float32List(46);

    // PhysicsPoint2PointConstraint
    // v3
    Vector3 get pivotA {
      var data = _data;
      return new Vector3(data[0], data[1], data[2]);
    }
    void set pivotA(Vector3 pivotA_) {
      var data = _data;
      data[0] = pivotA_[0];
      data[1] = pivotA_[1];
      data[2] = pivotA_[2];
    }
    // v3
    Vector3 get pivotB {
      var data = _data;
      return new Vector3(data[3], data[4], data[5]);
    }
    void set pivotB(Vector3 pivotB_) {
      var data = _data;
      data[3] = pivotB_[0];
      data[4] = pivotB_[1];
      data[5] = pivotB_[2];
    }
    //Float32List pivotB;//: any; // v3

    double get force {
      return _data[30];
    }
    void set force(double force_) {
      _data[30] = force_;
    }

    double get damping {
        return _data[31];
    }
    void set damping(double damping_) {
        _data[31] = damping_;
    }

    double get impulseClamp {
      return _data[32];
    }
    void set impulseClampe(double impulseClamp_) {
      _data[32] = impulseClamp_;
    }

    bool get active => _active;
    void set active(bool active_) {
      var pc = this;
      if (active_ == pc.active) {
        // If already active, and in a world then allow re-setting to true
        // to update wakeTimeStamp.
        if (pc._world && active_) {
          pc._wakeTimeStamp = pc._world._timeStamp;
        }
      } else if (pc._world != null) {
        // If in a world, and not already active then wake the constraint.
        if (active_) {
          pc._world._wakeConstraint(pc);
        } else { // Otherwise force constraint to go to sleep.
          var list = pc._world._activeConstraints;
          //list[list.indexOf(pc)] = list[list.length - 1];
          //list.removeLast();
          _world._activeConstraints.remove(this);
          pc._active = false;
        }
      } else {
        pc._active = active_;
      }
    }



    //double force;//: number;
    //double damping;//: number;
    //double impulseClamp;//: number;

    WebGLPhysicsPoint2PointConstraint({
      WebGLPhysicsPrivateBody bodyA,
      WebGLPhysicsPrivateBody bodyB,
      Vector3 pivotA,
      Vector3 pivotB,
      double force: 0.3,
      double damping: 1.0,
      double impulseClamp: 0.0,
      bool active
    }) : super(bodyA: bodyA, bodyB: bodyB, active: active) {
        var c = this;
        //var pc = new WebGLPhysicsPrivatePoint2PointConstraint();
        //c._private = pc;

        // initConstraintProperties(c, params); start
        this.bodyA = bodyA;
        this.bodyB = bodyB;


        // initConstraintProperties(c, params); end

        var data = _data;
        // read/write with side effects
        data[0] = pivotA[0];
        data[1] = pivotA[1];
        data[2] = pivotA[2];
        /*Object.defineProperty(c, "pivotA", {
            get : function point2pointGetPivotA()
            {

            },
            set : function point2pointSetPivotA(pivotA)
            {

            },
            enumerable : true
        });*/

        // read/write with side effects
        // In the case that bodyB is not defined, we initialise pivot so that positional
        // error is 0.
        if (pivotB != null) {
            data[3] = pivotB[0];
            data[4] = pivotB[1];
            data[5] = pivotB[2];
        } else {
            //var pivotB = MathHelper.m43TransformPoint(bodyA._transform, pivotA);
            bodyA._transform.transformPoint(pivotA);
            data[3] = pivotB[0];
            data[4] = pivotB[1];
            data[5] = pivotB[2];
        }
        /*
        Object.defineProperty(c, "pivotB", {
            get : function point2pointGetPivotB()
            {
                var data = this._private.data;
                return VMath.v3Build(data[3], data[4], data[5]);
            },
            set : function point2pointSetPivotB(pivotB)
            {
                var data = this._private.data;
                data[3] = pivotB[0];
                data[4] = pivotB[1];
                data[5] = pivotB[2];
            },
            enumerable : true
        });*/

        // read/write with no immediate side effects, but getter/setter required.
        data[30] = force;
        /*
        Object.defineProperty(c, "force", {
            get : function point2pointGetForce()
            {
                return this._private.data[30];
            },
            set : function point2pointSetForce(force)
            {
                this._private.data[30] = force;
            },
            enumerable : true
        });*/

        // read/write with no immediate side effects, but getter/setter required.
        data[31] = damping;//(params.damping != null) ? params.damping : 1.0;


        // read/write with no immediate side effects, but getter/setter required.
        data[32] = impulseClamp;//(params.impulseClamp != null) ? params.impulseClamp : 0.0;
        /*Object.defineProperty(c, "impulseClamp", {
            get : function point2pointGetForce()
            {
                return this._private.data[32];
            },
            set : function point2pointSetForce(impulseClamp)
            {
                this._private.data[32] = impulseClamp;
            },
            enumerable : true
        });*/
    }
}

//WebGLPhysicsPoint2PointConstraint.prototype.type = ;


class WebGLPhysicsPrivatePoint2PointConstraint
{

    WebGLPhysicsCollisionObject bodyA;
    WebGLPhysicsCollisionObject bodyB;
    Float32List data;
    WebGLPhysicsWorld world;
    bool active;
    bool wakeTimeStamp;

    WebGLPhysicsPrivatePoint2PointConstraint() {
        // Initialise all properties that will ever be set on this object.
        this.bodyA = null;
        this.bodyB = null;

        // [0,  3) : pivotA (vector3)
        // [3,  6) : pivotB (vector3)
        // [6,  9) : relA   (vector3)
        // [9, 12) : relB   (vector3)
        // [12,21) : skewA  (mat33)
        // [21,30) : skewB  (mat33)
        // [30,31) : force   (scalar)
        // [31,32) : damping (scalar)
        // [32,33) : clamp   (scalar)
        // [33,34) : gamma   (scalar)
        // [34,40) : K (symmetric mat33)
        //           [ K0 K1 K2 ]
        //    aka:   [ K1 K3 K4 ]
        //           [ K2 K4 K5 ]
        // [40,43) : jAcc (vector3)
        // [43,46) : bias (vector3)
        this.data = new Float32List(46);
        //return this;
    }

    void preStep(double timeStepRatio, double timeStep) {
      WebGLPhysicsPrivateBody bodyA = this.bodyA;
      WebGLPhysicsPrivateBody bodyB = this.bodyB;
        var data = this.data;

        // a0 = this.pivotA
        var a0 = data[0];
        var a1 = data[1];
        var a2 = data[2];

        // b0 = this.pivotB
        var b0 = data[3];
        var b1 = data[4];
        var b2 = data[5];

        // Compute relative coordinates of pivot points.
        //this.relA = VMath.m43TransformVector(this.bodyA.transform, this.pivotA);
        Matrix43 A = bodyA._transform;
        var ra0 = data[6] = (A[0] * a0) + (A[3] * a1) + (A[6] * a2);
        var ra1 = data[7] = (A[1] * a0) + (A[4] * a1) + (A[7] * a2);
        var ra2 = data[8] = (A[2] * a0) + (A[5] * a1) + (A[8] * a2);

        var rb0, rb1, rb2, B;
        if (bodyB != null) {
            B = bodyB._transform;

            //this.relB = VMath.m43TransformVector(this.bodyB.transform, this.pivotB);
            rb0 = data[9]  = (B[0] * b0) + (B[3] * b1) + (B[6] * b2);
            rb1 = data[10] = (B[1] * b0) + (B[4] * b1) + (B[7] * b2);
            rb2 = data[11] = (B[2] * b0) + (B[5] * b1) + (B[8] * b2);
        }

        //var skew = this.matrix;
        //this.m33BuildSkew(this.relA, skew);
        //VMath.m33Mul(skew, bodyA.inverseInertia, this.skewA);
        var I = bodyA._inverseInertia;
        data[12] = (-ra2 * I[3]) + (ra1 * I[6]);
        data[13] = (-ra2 * I[4]) + (ra1 * I[7]);
        data[14] = (-ra2 * I[5]) + (ra1 * I[8]);
        data[15] = (ra2 * I[0]) + (-ra0 * I[6]);
        data[16] = (ra2 * I[1]) + (-ra0 * I[7]);
        data[17] = (ra2 * I[2]) + (-ra0 * I[8]);
        data[18] = (-ra1 * I[0]) + (ra0 * I[3]);
        data[19] = (-ra1 * I[1]) + (ra0 * I[4]);
        data[20] = (-ra1 * I[2]) + (ra0 * I[5]);

        var mass_sum = bodyA._inverseMass + (bodyB != null ? bodyB._inverseMass : 0.0);
        //VMath.m33BuildIdentity(K);
        //VMath.m33ScalarMul(K, mass_sum, K);
        //this.m33Sub(K, VMath.m33Mul(this.skewA, skew), K);
        var K0 = mass_sum + (data[13] * -ra2) + (data[14] * ra1);
        var K3 = mass_sum + (data[15] * ra2)  + (data[17] * -ra0);
        var K5 = mass_sum + (data[18] * -ra1) + (data[19] * ra0);
        var K1 = (data[12] * ra2)  + (data[14] * -ra0);
        var K2 = (data[12] * -ra1) + (data[13] * ra0);
        var K4 = (data[15] * -ra1) + (data[16] * ra0);

        if (bodyB != null) {
            //this.m33BuildSkew(this.relB, skew);
            //VMath.m33Mul(skew, bodyB.inverseInertia, this.skewB);
            //this.m33Sub(K, VMath.m33Mul(this.skewB, skew), K);
            I = bodyB._inverseInertia;
            data[21] = (-rb2 * I[3]) + (rb1 * I[6]);
            data[22] = (-rb2 * I[4]) + (rb1 * I[7]);
            data[23] = (-rb2 * I[5]) + (rb1 * I[8]);
            data[24] = (rb2 * I[0]) + (-rb0 * I[6]);
            data[25] = (rb2 * I[1]) + (-rb0 * I[7]);
            data[26] = (rb2 * I[2]) + (-rb0 * I[8]);
            data[27] = (-rb1 * I[0]) + (rb0 * I[3]);
            data[28] = (-rb1 * I[1]) + (rb0 * I[4]);
            data[29] = (-rb1 * I[2]) + (rb0 * I[5]);

            K0 += (data[22] * -rb2) + (data[23] * rb1);
            K3 += (data[24] * rb2)  + (data[26] * -rb0);
            K5 += (data[27] * -rb1) + (data[28] * rb0);
            K1 += (data[21] * rb2)  + (data[23] * -rb0);
            K2 += (data[21] * -rb1) + (data[22] * rb0);
            K4 += (data[24] * -rb1) + (data[25] * rb0);
        }

        // Soft constraint physics (Based on Nape physics soft constraints).
        //
        // We are given this.force in constraint parameters.
        //   So we must compute suitable omega instead.
        var force = data[30];
        var omega = (2 / timeStep * force * data[31]) / (1 - force);

        var gk = force / (omega * omega);
        var ig = 1 / (1 + gk);
        data[33] = 1 - (gk * ig);

        //VMath.m33Inverse(K, K);
        //VMath.m33ScalarMul(K, ig, K);
        var i0 = ((K3 * K5) - (K4 * K4));
        var i1 = ((K2 * K4) - (K1 * K5));
        var i2 = ((K1 * K4) - (K2 * K3));
        var idet = ig / ((K0 * i0) + (K1 * i1) + (K2 * i2));

        data[34] = (idet * i0);
        data[35] = (idet * i1);
        data[36] = (idet * i2);
        data[37] = (idet * ((K0 * K5) - (K2 * K2)));
        data[38] = (idet * ((K1 * K2) - (K0 * K4)));
        data[39] = (idet * ((K0 * K3) - (K1 * K1)));

        // positional error
        var C0 = ra0 + A[9];
        var C1 = ra1 + A[10];
        var C2 = ra2 + A[11];
        if (bodyB != null)
        {
            C0 -= rb0 + B[9];
            C1 -= rb1 + B[10];
            C2 -= rb2 + B[11];
        } else {
            C0 -= b0;
            C1 -= b1;
            C2 -= b2;
        }

        // soft constraint bias.
        var scale = -force / timeStep;
        data[43] = (C0 * scale);
        data[44] = (C1 * scale);
        data[45] = (C2 * scale);

        // scale cached impulse for change in time step.
        data[40] *= timeStepRatio;
        data[41] *= timeStepRatio;
        data[42] *= timeStepRatio;
    }

    void applyCachedImpulses() {
        var data = this.data;

        // var j = this.jAcc
        var j0 = data[40];
        var j1 = data[41];
        var j2 = data[42];

        var bodyA = this.bodyA;
        var vel = bodyA._velocity;
        var imass = bodyA._inverseMass;
        vel[0] += (j0 * imass);
        vel[1] += (j1 * imass);
        vel[2] += (j2 * imass);

        //var I = this.skewA;
        vel[3] -= ((data[12] * j0) + (data[15] * j1) + (data[18] * j2));
        vel[4] -= ((data[13] * j0) + (data[16] * j1) + (data[19] * j2));
        vel[5] -= ((data[14] * j0) + (data[17] * j1) + (data[20] * j2));

        var bodyB = this.bodyB;
        if (bodyB != null) {
            vel = bodyB._velocity;
            imass = bodyB._inverseMass;
            vel[0] -= (j0 * imass);
            vel[1] -= (j1 * imass);
            vel[2] -= (j2 * imass);

            //I = this.skewB;
            vel[3] += ((data[21] * j0) + (data[24] * j1) + (data[27] * j2));
            vel[4] += ((data[22] * j0) + (data[25] * j1) + (data[28] * j2));
            vel[5] += ((data[23] * j0) + (data[26] * j1) + (data[29] * j2));
        }
    }

    void computeAndApplyImpulses() {
        var bodyA = this.bodyA;
        var bodyB = this.bodyB;
        var data = this.data;

        // jAcc = this.jAcc
        var jAcc0 = data[40];
        var jAcc1 = data[41];
        var jAcc2 = data[42];

        // velocity bias, minus the relative velocity at pivot points
        // stored in (l0, l1, l2)
        var vel1 = bodyA._velocity;
        //var rel = this.relA;
        // l = bias - (vel1 + ang1 cross rel)
        var l0 = data[43] - (vel1[0] + (vel1[4] * data[8]) - (vel1[5] * data[7]));
        var l1 = data[44] - (vel1[1] + (vel1[5] * data[6]) - (vel1[3] * data[8]));
        var l2 = data[45] - (vel1[2] + (vel1[3] * data[7]) - (vel1[4] * data[6]));

        var vel2;
        if (bodyB != null) {
            vel2 = bodyB._velocity;
            //rel = this.relB;
            // l += vel2 + ang2 cross rel
            l0 += (vel2[0] + (vel2[4] * data[11]) - (vel2[5] * data[10]));
            l1 += (vel2[1] + (vel2[5] * data[9])  - (vel2[3] * data[11]));
            l2 += (vel2[2] + (vel2[3] * data[10]) - (vel2[4] * data[9]));
        }

        // compute, and accumulate impulse into (jAcc0, jAcc1, jAcc2)
        var gamma = data[33];
        //var K = this.K;
        // jAcc = jAcc * gamma + K * l
        jAcc0 = (jAcc0 * gamma) + (data[34] * l0) + (data[35] * l1) + (data[36] * l2);
        jAcc1 = (jAcc1 * gamma) + (data[35] * l0) + (data[37] * l1) + (data[38] * l2);
        jAcc2 = (jAcc2 * gamma) + (data[36] * l0) + (data[38] * l1) + (data[39] * l2);

        var clamp = data[32];
        if (clamp != 0) {
            // clamp accumulated impulse.
            var jlsq = (jAcc0 * jAcc0) + (jAcc1 * jAcc1) + (jAcc2 * jAcc2);
            if (jlsq > clamp * clamp) {
                jlsq = clamp / Math.sqrt(jlsq);
                jAcc0 *= jlsq;
                jAcc1 *= jlsq;
                jAcc2 *= jlsq;
            }
        }

        // compute impulse to apply, and store new cached impulse.
        var j0 = jAcc0 - data[40];
        var j1 = jAcc1 - data[41];
        var j2 = jAcc2 - data[42];
        data[40] = jAcc0;
        data[41] = jAcc1;
        data[42] = jAcc2;

        // Apply impulse.
        var imass = bodyA._inverseMass;
        vel1[0] += (j0 * imass);
        vel1[1] += (j1 * imass);
        vel1[2] += (j2 * imass);

        //var I = this.skewA;
        vel1[3] -= ((data[12] * j0) + (data[15] * j1) + (data[18] * j2));
        vel1[4] -= ((data[13] * j0) + (data[16] * j1) + (data[19] * j2));
        vel1[5] -= ((data[14] * j0) + (data[17] * j1) + (data[20] * j2));

        if (bodyB != null) {
            imass = bodyB._inverseMass;
            vel2[0] -= (j0 * imass);
            vel2[1] -= (j1 * imass);
            vel2[2] -= (j2 * imass);

            //I = this.skewB;
            vel2[3] += ((data[21] * j0) + (data[24] * j1) + (data[27] * j2));
            vel2[4] += ((data[22] * j0) + (data[25] * j1) + (data[28] * j2));
            vel2[5] += ((data[23] * j0) + (data[26] * j1) + (data[29] * j2));
        }
    }
}