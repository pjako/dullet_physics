part of dullet_physics;

//
// WebGL Physics Collision Object
//
class WebGLPhysicsCollisionObject //implements PhysicsCollisionObject
{
    static const version = 1;
    WebGLPhysicsPrivateBody _private;

    // From PhysicsCollisionObject
    //var transform; // m43
    /*WebGL*/WebGLPhysicsShape get shape => _shape;
    WebGLPhysicsShape _shape;
    //int group;  // getter for _private.group
    //int mask;   // getter for _private.mask
    //double friction; // getter for _private.friction
    //double restitution; // getter for _private.restitution
    //bool kinematic; // getter for _private.kinematic
    dynamic userData;

    Float32List inverseInertia;
    double inverseMass;
    Float32List velocity;

    WebGLPhysicsWorld world;

    WebGLPhysicsContactCallbacks contactCallbacks;

    bool sweepFrozen;

    //_private: WebGLPhysicsPrivateBody;
    //_public: any;  // TODO: what's this.  Seems to be refered to.

    // TODO: make this private
    static final sharedInverseInertiaLocal = new Vector3.zero();
    static final sharedInverseInertia = new Float32List(9);

    calculateExtents(Vector3 extents){
        this._private.calculateExtents(extents);
    }

    calculateTransform(transform, origin)
    {
        var privateTransform = this._private.transform;
        if (origin)
        {
            VMath.m43NegOffset(privateTransform, origin, transform);
        }
        else
        {
            transform[0] = privateTransform[0];
            transform[1] = privateTransform[1];
            transform[2] = privateTransform[2];
            transform[3] = privateTransform[3];
            transform[4] = privateTransform[4];
            transform[5] = privateTransform[5];
            transform[6] = privateTransform[6];
            transform[7] = privateTransform[7];
            transform[8] = privateTransform[8];
            transform[9] = privateTransform[9];
            transform[10] = privateTransform[10];
            transform[11] = privateTransform[11];
        }
    }

    clone() {
        return WebGLPhysicsCollisionObject.create(this);
    }
    int get group => _group;
    int _group;
    int get mask => _mask;
    int _mask;
    Float32List get transform => MathHelper.m43Copy(_private.transform);
    void set transform(Float32list transform) {
      var pr = this._private;
      // can only set transform if kinematic, or else for non kinematic IF NOT in a world.
      if (pr.kinematic || !pr.world) {
        MathHelper.m43Copy(transform, pr.transform);
        if (pr.world) {
          pr.world.wakeBody(pr);
        }
      }
    }
    double get friction => _private.friction;
    void set friction(double friction) {
      var pr = this._private;
      pr.friction = friction;

      // Invalidate arbiter friction values.
      var arbiters = pr.arbiters;
      var i;
      var limit = arbiters.length;
      for (i = 0; i < limit; i += 1) {
        arbiters[i].invalidateParameters();
      }
    }
    double get restitution => _private.restitution;
    void set restitution(double restitution) {
      var pr = this._private;
      pr.restitution = restitution;

      // Invalidate arbiter restitution values.
      var arbiters = pr.arbiters;
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
      ContactCallback onPreSolveContact,
      ContactCallback onAddedContacts,
      ContactCallback onProcessedContacts,
      ContactCallback onRemovedContacts,
      int group: WebGLPhysicsDevice.FILTER_STATIC,
      int mask: WebGLPhysicsDevice.FILTER_ALL ^ WebGLPhysicsDevice.FILTER_STATIC,
      bool kinematic: false,
      dynamic userData}) {
        var rets = this;
        var s = new WebGLPhysicsPrivateBody(params, rets);
        rets._private = s;

        //read/write, no side effects
        rets.userData = userData;

        // read only, no getter needed
        rets._shape = shape;

        _group = group;
        _mask = mask;

        _kinematic = kinematic;

        //--------------------------------
        // set private collision object properties

        s.group = group;
        s.mask = mask;

        s.kinematic = kinematic;
        s.fixedRotation = !kinematic;

        s.mass = 0.0;
        s.inverseMass = 0.0;

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
        if (onPreSolveContact != null || onAddedContacts != null || onProcessedContacts != null || onRemovedContacts != null)
        {
            s.contactCallbacks = new WebGLPhysicsContactCallbacks(
                onAddedContacts: onAddedContacts,
                onProcessedContacts: onProcessedContacts,
                onRemovedContacts: onRemovedContacts,
                mask: mask);
        } else {
            s.contactCallbacks = null;
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
                return VMath.m43Copy(this._private.transform);
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
                return this._private.friction;
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
                return this._private.restitution;
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
