part of dullet_physics;



//
// WebGLPhysicsRigidBody
//

// TODO: inherit from WebGLPhysicsCollisionObject

class WebGLPhysicsRigidBody extends WebGLPhysicsPrivateBody /*implements PhysicsRigidBody*/ {
  static const int version = 1;
  dynamic userData;
  bool get active => _active;
  void set active(bool active_) {
    var pr = this;
    if (active_ == pr._active)
     {
      // If already active, and in a world then allow re-settnig to true
      // to update wakeTimeStamp.
      if (pr._world != null && active_)
       {
        pr._wakeTimeStamp = pr._world._timeStamp;
      }
    } else {
      if (pr._world != null) {
        // If in a world, and not already active then wake the body.
        if (active_) {
          pr._world._wakeBody(pr);
        } else { // Otherwise force body to go to sleep.
          pr._world._activeBodies.removeElement(pr);
          /*var list = pr.world.activeBodies;

          list[list.indexOf(pr)] = list[list.length - 1];
          list.pop();*/
          pr._active = false;

          // force any arbiters to be deactivated also.
          var arbiters = pr._arbiters;
          var n;
          var maxN = arbiters.length;
          for (n = 0; n < maxN; n += 1) {
            var arb = arbiters[n];
            if (!arb._active) {
              continue;
            }
            arb._active = false;
            var worldList = pr._world._activeArbiters;
            worldList[worldList.indexOf(arb)] = worldList[worldList.length - 1];
            worldList.pop();
          }

          // sync with broadphase
          pr._world._syncBody(pr);
        }
      } else {
        pr._active = active_;
      }
    }
  }

  Matrix43 get transform => _transform.clone();
  void set transform(Matrix43 transform) {
    var pr = this;
    transform.copyInto(pr._transform);
    //MathHelper.m43Copy(transform, pr._transform);

    // Ensure any arbiter's have their skipDiscreteCollisions flags set to false as
    // new contacts 'will' be needed.
    var arbiters = pr._arbiters;
    var i;
    var limit = arbiters.length;
    for (i = 0; i < limit; i += 1) {
      arbiters[i].skipDiscreteCollisions = false;
    }
  }
  int get group => _group; // getter for group
  int get mask => _mask; // getter for mask
  //int friction; // getter for friction
  double get friction => _friction;
  void set friction(double fricton) {
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



  //int restitution; // getter for restitution
  double get restitution => _restitution;
  void set restitution(double restitution) {
    var pr = this;
    pr._restitution = restitution;

    // Invalidate arbiter restitution values.
    var arbiters = pr._arbiters;
    var i;
    var limit = arbiters.length;
    for (i = 0; i < limit; i += 1)
     {
      arbiters[i].invalidateParameters();
    }
  }
  //booMematic; // getter for kinematic

  // From WebGLPhysicsRigidBody
  //Float32List linearVelocity; // v3
  //Float32List angularVelocity; // v3
  Vector3 get angularVelocity {
    var vel = this._velocity;
    return new Vector3(vel.storage[3], vel.storage[4], vel.storage[5]);
  }
  void set angularVelocity(Vector3 angularVelocity_) {
    var vel = this._velocity;
    vel.storage[3] = angularVelocity_.storage[0];
    vel.storage[4] = angularVelocity_.storage[1];
    vel.storage[5] = angularVelocity_.storage[2];
  }

  double get linearDamping => _linearDamping;
  void set linearDamping(linearDamping) {
    _linearDamping = linearDamping;
  }
  double get angularDamping => _angularDamping;
  void set angularDamping(angularDamping) {
    _angularDamping = angularDamping;
  }

  // read only, no getter needed
  bool get kinematic => _kinematic;
  //bool _kinematic;

  // read only, no getter needed
  double get mass => _mass;
  //double _mass;

  // read only, getter needed for unique vector.
  // this value isn't used internally so is kept in a closure just for this getter.
  //
  Vector3 get inertia => _inertia;
  Vector3 _inertia;


  //double linearDamping;
  //double angularDamping;
  //double mass;
  //Float32List inertia; // v3

  //WebGLPhysicsPrivateBody _private;


  WebGLPhysicsShape get shape => _shape;

  //calculateExtents = WebGLPhysicsCollisionObject.prototype.calculateExtents;

  //calculateTransform = WebGLPhysicsCollisionObject.prototype.calculateTransform;

  /*WebGLPhysicsRigidBody clone() {
        return WebGLPhysicsRigidBody.create(this);
    }*/

  WebGLPhysicsRigidBody clone() {
    return new WebGLPhysicsRigidBody(
    shape: shape,
     transform: _transform.clone(),
     //linearVelocity: _linearVelocity,
    //angularVelocity: _angularVelocity,
    userData: userData,
     mask: mask,
     group: group,
     kinematic: kinematic,
     inertia: inertia,
     mass: mass,
     friction: friction,
     restitution: restitution,
     linearDamping: linearDamping,
     angularDamping: angularDamping,
     //frozen: _frozen,
    active: active
    //fixedRotation: fixedRotation,
    //permitSleep: permitSleep,
    //onPreSolveContact: _onPreSolveContact,
    //onAddedContacts,
    //onProcessedContacts,
    //onRemovedContacts
    );
  }


  WebGLPhysicsRigidBody({
  WebGLPhysicsShape shape, 
  Matrix43 transform, 
  Vector3 linearVelocity, 
  Vector3 angularVelocity, 
  dynamic this.userData, 
  int mask: WebGLPhysicsDevice.FILTER_ALL, 
  int group: WebGLPhysicsDevice.FILTER_DYNAMIC, 
  bool kinematic: false, 
  Vector3 inertia, 
  double mass: 0.5, 
  double friction: 0.5, 
  double restitution: 0.0, 
  double linearDamping: 0.0, 
  double angularDamping: 0.0, 
  bool frozen, 
  bool active: true, 
  bool fixedRotation: false, 
  bool permitSleep, 
  ContactCallback onPreSolveContact, 
  ContactCallback onAddedContacts, 
  ContactCallback onProcessedContacts, 
  ContactCallback onRemovedContacts})
      : super(
      //WebGLPhysicsCollisionObject publicObject,
      shape,
       transform,
       linearVelocity,
       angularVelocity,
       friction,
       restitution,
       linearDamping,
       angularDamping) {
    //retr._private = r;
    if (inertia != null) {
      _inertia = inertia.clone();
    } else {
      _inertia = shape._inertia.clone().scale(mass);
    }
    var r = this;

    _kinematic = kinematic;
    // ------------------------------
    // initialise private properties of RigidBody.

    _group = group;
    _mask = mask;
    _active = (active != null) ? active : (frozen != null) ? (frozen) : true;

    _kinematic = kinematic;
    _fixedRotation = kinematic || fixedRotation;

    _inverseInertiaLocal = (r._fixedRotation ? new Vector3.zero() : new Vector3(1.0 / inertia[0], 1.0 / inertia[1], 1.0 / inertia[2]));
    _inverseInertia = new Matrix3.identity();//MathHelper.m33BuildIdentity();

    _mass = mass;
    _inverseMass = (kinematic ? 0.0 : (1.0 / r.mass));

    _collisionObject = false;

    // Kinematic object is not permitted to sleep in the normal sense.
    _permitSleep = (permitSleep != null) ? permitSleep : (!kinematic);

    // Kinematic object is not subject to manipulation by continous collisions.
    _sweepFrozen = kinematic;

    // prepare for contact callbacks
    if (onPreSolveContact != null ||
     onAddedContacts != null ||
     onProcessedContacts != null ||
     onRemovedContacts != null) {
      _contactCallbacks = new WebGLPhysicsContactCallbacks(
      mask: mask,
       trigger: false,
       onAddedContacts: onAddedContacts,
       onPreSolveContact: onPreSolveContact,
       onProcessedContacts: onProcessedContacts,
       onRemovedContacts: onRemovedContacts);
    } else {
      _contactCallbacks = null;
    }
  }
}

//var r = new WebGLPhysicsPrivateBody(params, retr);


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

//var kinematic = (params.kinematic !== undefined) ? params.kinematic : false;
//var mass = (params.mass !== undefined) ? params.mass : 1.0;
//var inertia = (params.inertia ? VMath.v3Copy(params.inertia) : VMath.v3ScalarMul(params.shape.inertia, mass));


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
