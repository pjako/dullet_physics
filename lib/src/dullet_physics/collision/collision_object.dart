part of dullet_physics;

//
// WebGL Physics Collision Object
//
class WebGLPhysicsCollisionObject extends WebGLPhysicsPrivateBody// PhysicsCollisionObject
{
    static const version = 1;
    //WebGLPhysicsPrivateBody _private;

    // From PhysicsCollisionObject
    //var transform; // m43
    /*WebGL*/WebGLPhysicsShape get shape => _shape;
    //WebGLPhysicsShape _shape;
    //int group;  // getter for group
    //int mask;   // getter for mask
    //double friction; // getter for friction
    //double restitution; // getter for restitution
    //bool kinematic; // getter for kinematic
    dynamic userData;

    //Float32List inverseInertia;
    //double inverseMass;
    //Float32List velocity;

    //WebGLPhysicsWorld world;

    WebGLPhysicsContactCallbacks contactCallbacks;

    //bool sweepFrozen;

    //_private: WebGLPhysicsPrivateBody;
    //_public: any;  // TODO: what's this.  Seems to be refered to.

    // TODO: make this private
    static final Vector3 sharedInverseInertiaLocal = new Vector3.zero();
    static final Matrix3 sharedInverseInertia = new Matrix3.identity();

    calculateExtents(Aabb3 extents){
        this._calculateExtents(extents);
    }

    clone() {
        return new WebGLPhysicsCollisionObject(
            shape: _shape,
            /*onPreSolveContact: _onPreSolveContact,
            onAddedContacts: _onAddedContacts,
            onProcessedContacts: _onProcessedContacts,
            onRemovedContacts: _onRemovedContacts,*/
            group: _group,
            mask: _mask,
            kinematic: _kinematic,
            userData: userData);
    }
    int get group => _group;
    int _group;
    int get mask => _mask;
    int _mask;
    Matrix43 get transform => _transform.clone();
    void set transform(Matrix43 transform) {
      var pr = this;
      // can only set transform if kinematic, or else for non kinematic IF NOT in a world.
      if (pr._kinematic || !pr._world) {
        MathHelper.m43Copy(pr._transform, pr._transform);
        if (pr._world != null) {
          pr._world._wakeBody(pr);
        }
      }
    }
    double get friction => _friction;
    void set friction(double friction) {
      var pr = this;
      pr._friction = friction;

      // Invalidate arbiter friction values.
      var arbiters = pr._arbiters;
      var i;
      var limit = arbiters.length;
      for (i = 0; i < limit; i += 1) {
        arbiters[i].invalidateParameters();
      }
    }
    double get restitution => _restitution;
    void set restitution(double restitution) {
      var pr = this;
      pr._restitution = restitution;

      // Invalidate arbiter restitution values.
      var arbiters = pr._arbiters;
      var i;
      var limit = arbiters.length;
      for (i = 0; i < limit; i += 1) {
        arbiters[i].invalidateParameters();
      }
    }
    bool _kinematic;
    bool get kinematic => _kinematic;
    WebGLPhysicsCollisionObject({
      WebGLPhysicsShape shape,
      Matrix43 transform,
      double friction: 0.5,
      double restitution: 0.0,
      Vector3 linearVelocity,
      Vector3 angularVelocity,
      double linearDamping: 0.0,
      double angularDamping: 0.0,
      ContactCallback onPreSolveContact,
      ContactCallback onAddedContacts,
      ContactCallback onProcessedContacts,
      ContactCallback onRemovedContacts,
      int group: WebGLPhysicsDevice.FILTER_STATIC,
      int mask: WebGLPhysicsDevice.FILTER_ALL ^ WebGLPhysicsDevice.FILTER_STATIC,
      bool kinematic: false,
      dynamic userData}) : super(
          shape: shape,
          transform: transform,
          linearVelocity: linearVelocity,
          angularVelocity: angularVelocity,
          friction: friction) {
        var rets = this;

        // Needs data!

        //var s = new WebGLPhysicsPrivateBody(params, rets);
        //rets._private = s;

        //read/write, no side effects
        rets.userData = userData;

        // read only, no getter needed
        rets._shape = shape;

        _group = group;
        _mask = mask;

        _kinematic = kinematic;

        //--------------------------------
        // set private collision object properties

        _group = group;
        _mask = mask;

        _kinematic = kinematic;
        _fixedRotation = !kinematic;

        _mass = 0.0;
        _inverseMass = 0.0;

        _inverseInertiaLocal = WebGLPhysicsCollisionObject.sharedInverseInertiaLocal;
        _inverseInertia = WebGLPhysicsCollisionObject.sharedInverseInertia;

        _collisionObject = true;

        // Kinematic/Static object is not permitted to sleep in the normal sense.
        _permitSleep = false;
        // Kinematic/Static objects are not subject to manipulation by continuous
        // collision detection.
        _sweepFrozen = true;

        // Object default active state is true iff object is kinematic.
        // static object is always 'inactive'
        _active = kinematic;

        // prepare for contact callbacks
        if (onPreSolveContact != null || onAddedContacts != null || onProcessedContacts != null || onRemovedContacts != null)
        {
            _contactCallbacks = new WebGLPhysicsContactCallbacks(
                onAddedContacts: onAddedContacts,
                onProcessedContacts: onProcessedContacts,
                onRemovedContacts: onRemovedContacts,
                mask: mask);
        } else {
            _contactCallbacks = null;
        }

        //return rets;
    }
/*
    static create(params: any): WebGLPhysicsCollisionObject
    {
        var rets = new WebGLPhysicsCollisionObject();
        var s = new WebGLPhysicsPrivateBody(params, rets);
        rets._private = s;

        //read/write, no side effects
        rets.userData = ("userData" in params) ? params.userData : null;

        // read only, no getter needed
        Object.defineProperty(rets, "shape", {
            value : params.shape,
            enumerable : true
        });

        var kinematic = (params.kinematic !== undefined) ? params.kinematic : false;

        // read/write, side effects
        Object.defineProperty(rets, "transform", {
            get : function collisionObjectGetTransform()
            {
                return VMath.m43Copy(this.transform);
            },
            set : function collisionObjectSetTransform(transform)
            {
                var pr = this._private;
                // can only set transform if kinematic, or else for non kinematic IF NOT in a world.
                if (pr.kinematic || !pr.world)
                {
                    VMath.m43Copy(transform, pr.transform);
                    if (pr.world)
                    {
                        pr.world.wakeBody(pr);
                    }
                }
            },
            enumerable : true
        });

        var group = (params.group !== undefined) ? params.group : WebGLPhysicsDevice.prototype.FILTER_STATIC;
        // read only, no getter needed
        Object.defineProperty(rets, "group", {
            value : group,
            enumerable : true
        });

        /*jshint bitwise: false*/
        var mask  = (params.mask !== undefined) ? params.mask  :
            (WebGLPhysicsDevice.prototype.FILTER_ALL ^ WebGLPhysicsDevice.prototype.FILTER_STATIC);
        /*jshint bitwise: true*/
        // read only, no getter needed
        Object.defineProperty(rets, "mask", {
            value : mask,
            enumerable : true
        });

        // read/write, side effects needed
        Object.defineProperty(rets, "friction", {
            get : function collisionObjectGetFriction()
            {
                return this.friction;
            },
            set : function collisionObjectSetFriction(friction)
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

        // read/write, side effects needed
        Object.defineProperty(rets, "restitution", {
            get : function collisionObjectGetFriction()
            {
                return this.restitution;
            },
            set : function collisionObjectSetFriction(restitution)
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
        });

        // read only, no getter needed
        Object.defineProperty(rets, "kinematic", {
            value : kinematic,
            enumerable : true
        });

        //--------------------------------
        // set private collision object properties

        s.group = group;
        s.mask = mask;

        s.kinematic = kinematic;
        s.fixedRotation = !kinematic;

        s.mass = 0;
        s.inverseMass = 0;

        s.inverseInertiaLocal = WebGLPhysicsCollisionObject.sharedInverseInertiaLocal;
        s.inverseInertia = WebGLPhysicsCollisionObject.sharedInverseInertia;

        s.collisionObject = true;

        // Kinematic/Static object is not permitted to sleep in the normal sense.
        s.permitSleep = false;
        // Kinematic/Static objects are not subject to manipulation by continuous
        // collision detection.
        s.sweepFrozen = true;

        // Object default active state is true iff object is kinematic.
        // static object is always 'inactive'
        s.active = kinematic;

        // prepare for contact callbacks
        if (params.onPreSolveContact ||
            params.onAddedContacts ||
            params.onProcessedContacts ||
            params.onRemovedContacts)
        {
            s.contactCallbacks = new WebGLPhysicsContactCallbacks(params, mask);
        }
        else
        {
            s.contactCallbacks = null;
        }

        return rets;
    }*/
}
