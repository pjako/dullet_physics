part of dullet_physics;



//
// WebGLPhysicsRigidBody
//

// TODO: inherit from WebGLPhysicsCollisionObject

class WebGLPhysicsRigidBody /*implements PhysicsRigidBody*/ {
    static const int version = 1;

    double wakeTimeStamp;
    WebGLPhysicsIsland island;
    List<WebGLPhysicsArbiter> arbiters;

    bool get active => _private.active;
    void set active(bool active_) {
      var pr = this._private;
      if (active_ == pr.active)
      {
        // If already active, and in a world then allow re-settnig to true
        // to update wakeTimeStamp.
        if (pr.world && active_)
        {
          pr.wakeTimeStamp = pr.world.timeStamp;
        }
      } else if (pr.world) {
        // If in a world, and not already active then wake the body.
        if (active_) {
          pr.world.wakeBody(pr);
        } else { // Otherwise force body to go to sleep.
          pr.world.activeBodies.removeElement(pr);
          /*var list = pr.world.activeBodies;

          list[list.indexOf(pr)] = list[list.length - 1];
          list.pop();*/
          pr.active = false;

          // force any arbiters to be deactivated also.
          var arbiters = pr.arbiters;
          var n;
          var maxN = arbiters.length;
          for (n = 0; n < maxN; n += 1) {
            var arb = arbiters[n];
            if (!arb.active) {
              continue;
            }
            arb.active = false;
            var worldList = pr.world.activeArbiters;
            worldList[worldList.indexOf(arb)] = worldList[worldList.length - 1];
            worldList.pop();
          }

          // sync with broadphase
          pr.world.syncBody(pr);
        }
      } else {
        pr.active = active_;
      }
    }

    // From PhysicsCollisionObject
    //Float32List transform; // m43
    Float32List get transform => new Float32List.fromList(_private.transform);
    void set transform(Float32List transform) {
      var pr = this._private;
      MathHelper.m43Copy(transform, pr.transform);

      // Ensure any arbiter's have their skipDiscreteCollisions flags set to false as
      // new contacts 'will' be needed.
      var arbiters = pr.arbiters;
      var i;
      var limit = arbiters.length;
      for (i = 0; i < limit; i += 1) {
        arbiters[i].skipDiscreteCollisions = false;
      }
    }
    int get group => _private.group;  // getter for _private.group
    int get mask => _private.mask;   // getter for _private.mask
    //int friction; // getter for _private.friction
    double get friction => _private.friction;
    void set friction(double fricton) {
      var pr = _private;
      pr.friction = friction;

      // Invalidate arbiter friction values.
      var arbiters = pr.arbiters;
      var i;
      var limit = arbiters.length;
      for (i = 0; i < limit; i += 1) {
        arbiters[i].invalidateParameters();
      }

    }



    //int restitution; // getter for _private.restitution
    double get restitution => _private.restitution;
    void set restitution(double restitution) {
      var pr = this._private;
      pr.restitution = restitution;

      // Invalidate arbiter restitution values.
      var arbiters = pr.arbiters;
      var i;
      var limit = arbiters.length;
      for (i = 0; i < limit; i += 1)
      {
        arbiters[i].invalidateParameters();
      }
    }
    //booMematic; // getter for _private.kinematic

    // From WebGLPhysicsRigidBody
    Float32List linearVelocity; // v3
    //Float32List angularVelocity; // v3
    Float32List get angularVelocity {
      var vel = this._private.velocity;
      return new Float32List.fromList(vel[3], vel[4], vel[5]);
    }
    void set angularVelocity(Float32List angularVelocity_) {
      var vel = this._private.velocity;
      vel[3] = angularVelocity_[0];
      vel[4] = angularVelocity_[1];
      vel[5] = angularVelocity_[2];
    }

    double get linearDamping => _private.linearDamping;
    void set linearDamping(linearDamping) {
      _private.linearDamping = linearDamping;
    }
    double get angularDamping => _private.angularDamping;
    void set angularDamping(angularDamping){
      _private.angularDamping = angularDamping;
    }

    // read only, no getter needed
    bool get kinematic => _kinematic;
    bool _kinematic;

    // read only, no getter needed
    double get mass => _mass;
    double _mass;

    // read only, getter needed for unique vector.
    // this value isn't used internally so is kept in a closure just for this getter.
    //
    Float32List get inertia => _inertia;
    Float32List _inertia;


    //double linearDamping;
    //double angularDamping;
    //double mass;
    //Float32List inertia; // v3

    WebGLPhysicsPrivateBody _private;
    dynamic userData;

    WebGLPhysicsShape get shape => _shape;
    var _shape;

    //calculateExtents = WebGLPhysicsCollisionObject.prototype.calculateExtents;

    //calculateTransform = WebGLPhysicsCollisionObject.prototype.calculateTransform;

    WebGLPhysicsRigidBody clone() {
        return WebGLPhysicsRigidBody.create(this);
    }

    WebGLPhysicsRigidBody({
      WebGLPhysicsShape shape,
      dynamic this.userData,
      int mask: WebGLPhysicsDevice.FILTER_ALL,
      int group: WebGLPhysicsDevice.FILTER_DYNAMIC,
      bool kinematic: false,
      double mass: 1.0,
      bool frozen,
      bool active,
      bool fixedRotation: false,
      bool permitSleep,
      onPreSolveContact,
      onAddedContacts,
      onProcessedContacts,
      onRemovedContacts}) {
        var retr = this;
        var r = new WebGLPhysicsPrivateBody(params, retr);
        retr._private = r;

        // read/write, no side effects
        //retr.userData = ("userData" in params) ? params.userData : null;

        // read only, no getter needed
        /*Object.defineProperty(retr, "shape", {
            value : params.shape,
            enumerable : true
        });*/

        // read/write, side effects
        /*Object.defineProperty(retr, "linearVelocity", {
            get : function rigidBodyGetVelocity()
            {
                var vel = this._private.velocity;
                return VMath.v3Build(vel[0], vel[1], vel[2]);
            },
            set : function rigidBodySetVelocity(linearVelocity)
            {
                var vel = this._private.velocity;
                vel[0] = linearVelocity[0];
                vel[1] = linearVelocity[1];
                vel[2] = linearVelocity[2];
            },
            enumerable : true
        });*/

        // read/write, side effects
        /*
        Object.defineProperty(retr, "angularVelocity", {
            get : function rigidBodyGetVelocity()
            {
                var vel = this._private.velocity;
                return VMath.v3Build(vel[3], vel[4], vel[5]);
            },
            set : function rigidBodySetVelocity(angularVelocity)
            {
                var vel = this._private.velocity;
                vel[3] = angularVelocity[0];
                vel[4] = angularVelocity[1];
                vel[5] = angularVelocity[2];
            },
            enumerable : true
        });*/

        // read/write, side effects
        /*
        Object.defineProperty(retr, "transform", {
            get : function rigidBodyGetTransform()
            {
                return VMath.m43Copy(this._private.transform);
            },
            set : function rigidBodySetTransform(transform)
            {
                var pr = this._private;
                VMath.m43Copy(transform, pr.transform);

                // Ensure any arbiter's have their skipDiscreteCollisions flags set to false as
                // new contacts 'will' be needed.
                var arbiters = pr.arbiters;
                var i;
                var limit = arbiters.length;
                for (i = 0; i < limit; i += 1)
                {
                    arbiters[i].skipDiscreteCollisions = false;
                }
            },
            enumerable : true
        });*/

        // read/write, side effects
        /*
        Object.defineProperty(retr, "active", {
            get : function rigidBodyGetActive()
            {
                return this._private.active;
            },
            set : function rigidBodySetActive(active)
            {
                var pr = this._private;
                if (active === pr.active)
                {
                    // If already active, and in a world then allow re-settnig to true
                    // to update wakeTimeStamp.
                    if (pr.world && active)
                    {
                        pr.wakeTimeStamp = pr.world.timeStamp;
                    }
                }
                else if (pr.world)
                {
                    // If in a world, and not already active then wake the body.
                    if (active)
                    {
                        pr.world.wakeBody(pr);
                    }
                    // Otherwise force body to go to sleep.
                    else
                    {
                        var list = pr.world.activeBodies;
                        list[list.indexOf(pr)] = list[list.length - 1];
                        list.pop();
                        pr.active = false;

                        // force any arbiters to be deactivated also.
                        var arbiters = pr.arbiters;
                        var n;
                        var maxN = arbiters.length;
                        for (n = 0; n < maxN; n += 1)
                        {
                            var arb = arbiters[n];
                            if (!arb.active)
                            {
                                continue;
                            }

                            arb.active = false;
                            var worldList = pr.world.activeArbiters;
                            worldList[worldList.indexOf(arb)] = worldList[worldList.length - 1];
                            worldList.pop();
                        }

                        // sync with broadphase
                        pr.world.syncBody(pr);
                    }
                }
                else
                {
                    pr.active = active;
                }
            },
            enumerable : true
        });*/

        // read only, no getter needed
        //var group = (params.group !== undefined) ? params.group : WebGLPhysicsDevice.prototype.FILTER_DYNAMIC;
        //this.group = group;
        /*
        var group_ = (group != null) ? params.group :
        Object.defineProperty(retr, "group", {
            value : group_,
            enumerable : true
        });*/

        // read only, no getter needed
        //var mask  = (params.mask  !== undefined) ? params.mask  : WebGLPhysicsDevice.prototype.FILTER_ALL;
        _kinematic = kinematic;
        //var kinematic = (params.kinematic !== undefined) ? params.kinematic : false;
        //var mass = (params.mass !== undefined) ? params.mass : 1.0;
        //var inertia = (params.inertia ? VMath.v3Copy(params.inertia) : VMath.v3ScalarMul(params.shape.inertia, mass));
        if(inertia != null) {
          _inertia = Float32List.fromList(inertia);
        } else {
          VMath.v3ScalarMul(shape.inertia, mass);
        }

        //this.mask = mask;
        /*
        Object.defineProperty(retr, "mask", {
            value : mask,
            enumerable : true
        });
        */

        // read/write, side effects
        /*
        Object.defineProperty(retr, "friction", {
            get : function rigidBodyGetFriction()
            {
                return this._private.friction;
            },
            set : function rigidBodySetFriction(friction)
            {
                var pr = this._private;
                pr.friction = friction;

                // Invalidate arbiter friction values.
                var arbiters = pr.arbiters;
                var i;
                var limit = arbiters.length;
                for (i = 0; i < limit; i += 1)
                {
                    arbiters[i].invalidateParameters();
                }
            },
            enumerable : true
        });
        */
        /*
        Object.defineProperty(retr, "restitution", {
            get : function rigidBodyGetRestitution()
            {
                return this._private.restitution;
            },
            set : function rigidBodySetRestitution(restitution)
            {
                var pr = this._private;
                pr.restitution = restitution;

                // Invalidate arbiter restitution values.
                var arbiters = pr.arbiters;
                var i;
                var limit = arbiters.length;
                for (i = 0; i < limit; i += 1)
                {
                    arbiters[i].invalidateParameters();
                }
            },
            enumerable : true
        });*/
/*
        // read/write, getters needed.
        Object.defineProperty(retr, "linearDamping", {
            get : function rigidBodyGetLinearDamping()
            {
                return this._private.linearDamping;
            },
            set : function rigidBodySetLinearDamping(linearDamping)
            {
                this._private.linearDamping = linearDamping;
            },
            enumerable : true
        });
        Object.defineProperty(retr, "angularDamping", {
            get : function rigidBodyGetLinearDamping()
            {
                return this._private.angularDamping;
            },
            set : function rigidBodySetLinearDamping(angularDamping)
            {
                this._private.angularDamping = angularDamping;
            },
            enumerable : true
        });

        // read only, no getter needed
        var kinematic = (params.kinematic !== undefined) ? params.kinematic : false;
        Object.defineProperty(retr, "kinematic", {
            value : kinematic,
            enumerable : true
        });

        // read only, no getter needed
        var mass = (params.mass !== undefined) ? params.mass : 1.0;
        Object.defineProperty(retr, "mass", {
            value : mass,
            enumerable : true
        });

        // read only, getter needed for unique vector.
        // this value isn't used internally so is kept in a closure just for this getter.
        var inertia = (params.inertia ? VMath.v3Copy(params.inertia) : VMath.v3ScalarMul(params.shape.inertia, mass));
        Object.defineProperty(retr, "inertia", {
            get : function rigidBodyGetInertia()
            {
                return VMath.v3Copy(inertia);
            },
            enumerable : true
        });*/

        // ------------------------------
        // initialise private properties of RigidBody.

        r.group = group;
        r.mask = mask;

        r.active = (active != null) ? active :
            (frozen != null) ? (frozen) : true;

        r.kinematic = kinematic;
        r.fixedRotation = kinematic || fixedRotation;

        r.inverseInertiaLocal = (r.fixedRotation ? new Float32List(3) : new Float32List.fromList(1.0 / inertia[0], 1.0 / inertia[1], 1.0 / inertia[2]));
        r.inverseInertia = MathHelper.m33BuildIdentity();

        r.mass = mass;
        r.inverseMass = (kinematic ? 0.0 : (1.0 / r.mass));

        r.collisionObject = false;

        // Kinematic object is not permitted to sleep in the normal sense.
        r.permitSleep = (permitSleep != null) ? permitSleep : (!kinematic);

        // Kinematic object is not subject to manipulation by continous collisions.
        r.sweepFrozen = kinematic;

        // prepare for contact callbacks
        if (onPreSolveContact ||
            onAddedContacts ||
            onProcessedContacts ||
            onRemovedContacts) {
            r.contactCallbacks = new WebGLPhysicsContactCallbacks(params, mask);
        } else {
            r.contactCallbacks = null;
        }

        //return retr;
    }
}