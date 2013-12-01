part of dullet_physics;



//
// WebGLPhysicsCharacter
//
class WebGLPhysicsCharacter  // implements PhysicsCharacter
{
  static const int version = 1;

  bool _crouch;
  bool _dead;
  Matrix43 _start; // m43
  Matrix43 _end; // m43
  WebGLPhysicsRigidBody _rigidBody;

  //Float32List velocity; // v3
  //Float32List position; // v3
  bool get crouch => _crouch;
  void set crouch(crouch) {
    var pc = this;
    if (!pc._dead && crouch != pc._crouch) {
      var rigidBody = pc._rigidBody;
      WebGLPhysicsCapsuleShape capsule = rigidBody.shape as WebGLPhysicsCapsuleShape;
      pc._crouch = crouch;
      if (crouch) {
        capsule.halfHeight = ((this.crouchHeight * 0.5) - this.radius);
        rigidBody.transform[10] -= ((this.height - this.crouchHeight) * 0.5);
      } else {
        capsule.halfHeight = ((this.height * 0.5) - this.radius);
        rigidBody.transform[10] += ((this.height - this.crouchHeight) * 0.5);
      }
      if (rigidBody._world) {
        rigidBody._world._wakeBody(rigidBody);
      }
    }
  }
  bool get dead => _dead;
  void set dead(bool dead) {
    var pc = this;
    if (pc._dead != dead) {
      var rigidBody = pc._rigidBody;
      WebGLPhysicsCapsuleShape capsule = rigidBody.shape as WebGLPhysicsCapsuleShape;

      pc._dead = dead;
      if (dead) {
        capsule.halfHeight = 0.0;
        rigidBody.transform[10] -= ((this.height - this.radius) * 0.5);
      } else {
        capsule.halfHeight = ((this.height * 0.5) - this.radius);
        rigidBody.transform[10] += ((this.height - this.radius) * 0.5);
      }

      if (rigidBody._world)
       {
        rigidBody._world._wakeBody(rigidBody);
      }
    }
  }
  double maxJumpHeight;
  dynamic userData;

  //WebGLPhysicsPrivateCharacter _private;


  bool _onGroundConvexCallback(hitResult) {
    // Less than cosine of 15 degrees.
    return hitResult.hitNormal[1] >= 0.26;
  }

  double get height => _height;
  double _height;
  double get radius => _radius;
  double _radius;
  double get stepHeight => _stepHeight;
  double _stepHeight;
  double get crouchHeight => _crouchHeight;
  double _crouchHeight;

  bool get onGround {
    var pc = this;
    var rigidBody = pc._rigidBody;

    if (rigidBody._world) {
      var pos = rigidBody.transform;
      var start = pc._start;
      var end = pc._end;

      start[9] = pos[9];
      start[10] = pos[10];
      start[11] = pos[11];

      end[9] = pos[9];
      end[10] = (pos[10] - (this.stepHeight * 0.5));
      end[11] = pos[11];

      var result = rigidBody._world._convexSweepTest(
      shape: rigidBody.shape,
       from: start,
       to: end,
       group: WebGLPhysicsDevice.FILTER_CHARACTER,
       callback: _onGroundConvexCallback);
      return (result != null);
    } else {
      return false;
    }
  }


  Float32List get position {
    var rigidBody = _rigidBody;
    return rigidBody._transform.position;
  }
  void set position(Float32List position) {
    var rigidBody = _rigidBody;
    var transform = rigidBody._transform;
    transform[9] = position[0];
    transform[10] = position[1];
    transform[11] = position[2];
    rigidBody.transform = rigidBody._transform; //invoke setter.
    rigidBody.active = true;
  }

  Vector3 get velocity {
    var rigidBody = this._rigidBody;
    return rigidBody._linearVelocity;
  }
  void set velocity(Vector3 velocity) {
    var rigidBody = this._rigidBody;
    rigidBody._linearVelocity = velocity;
    rigidBody.active = true;
  }


  int _group, _mask;
  int get group => _group;
  int get mask => _mask;


  Float32List start; // m43
  Float32List end; // m43
  WebGLPhysicsRigidBody rigidBody;

  void jump() {
    var pc = this;
    var rigidBody = pc._rigidBody;
    var world = rigidBody._world;
    if (world) {
      rigidBody._velocity[1] = Math.sqrt(-2.0 * (this.maxJumpHeight - this.stepHeight) * world.gravity[1]);
      rigidBody.transform[10] += this.stepHeight;
      world.wakeBody(rigidBody);
    }
  }

  void calculateExtents(Aabb3 extents) {
    _rigidBody._calculateExtents(extents);
  }

  void calculateTransform(Matrix43 transform, Float32List origin) {
    _rigidBody._calculateTransform(transform, origin);
  }

  WebGLPhysicsCharacter({
  dynamic userData, 
  int group: WebGLPhysicsDevice.FILTER_CHARACTER, 
  int mask: WebGLPhysicsDevice.FILTER_ALL, 
  double height, 
  double radius, 
  double mass, 
  Matrix43 transform, 
  Vector3 velocity, 
  double friction: 0.5, 
  double restitution: 0.0, 
  double crouchHeight, 
  double linearDamping: 0.0, 
  double angularDamping: 0.0, 
  double stepHeight: 0.35, 
  double maxJumpHeight: 1.0}) {
    var c = this;
    //var pc = new WebGLPhysicsPrivateCharacter();
    //c._private = pc;

    // Initialise all properties this object will ever hold.

    // Value of read/write property.
    _crouch = false;
    _dead = false;

    // Matrices re-used in all calls to onGround getter.
    _start = new Matrix43.identity();
    _end = new Matrix43.identity();

    // Reference to created RigidBody representing Character.
    _rigidBody = null;

    // read/write, no side effects.
    c.userData = userData;


    _height = height;
    _radius = radius;
    _stepHeight = stepHeight;
    c.maxJumpHeight = maxJumpHeight;

    _crouchHeight = (crouchHeight != null) ? crouchHeight : (0.5 * height);

    _group = group;
    _mask = mask;

    // Create inner RigidBody with Capsule shape.
    var capsule = new WebGLPhysicsCapsuleShape(
    radius: c.radius,
     height: (2.0 * ((c.height * 0.5) - c.radius)),
     margin: 0.0
    );

    var rigidBody = new WebGLPhysicsRigidBody(
    shape: capsule,
     mass: mass,
     transform: transform,
     linearVelocity: velocity,
     group: group,
     mask: mask,
     friction: friction,
     restitution: restitution,
     linearDamping: linearDamping,
     angularDamping: angularDamping,
     fixedRotation: true
    );

    // private (But internals like dynamics world need access through this object).
    _rigidBody = rigidBody;

  // Back reference to this public character, so that rayTests and
  // convexSweeps can in the case of intersecting a rigid body that
  // represents a character, return the character instead!

  // TODO: Here a WebGLPhysicsCharacter is assign to a property that
  // should be a WebGLPhysicsCollisionObject.  Can PhysicsCharacter
  // inherit from CollisionObject?

  //rigidBody._private._public = /*<WebGLPhysicsCollisionObject><any>*/c;
  }


}
/*
class WebGLPhysicsPrivateCharacter {
    static const int version = 1;



    WebGLPhysicsPrivateCharacter() {

    }


}*/
