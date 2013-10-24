part of dullet_physics;



//
// WebGLPhysicsCharacter
//
class WebGLPhysicsCharacter// implements PhysicsCharacter
{
    static const int version = 1;

    //Float32List velocity; // v3
    //Float32List position; // v3
    bool get crouch => _private.crouch;
    void set crouch(crouch) {
      var pc = this._private;
      if (!pc.dead && crouch != pc.crouch) {
        var rigidBody = pc.rigidBody._private;
        var capsule = rigidBody.shape;
        pc.crouch = crouch;
        if (crouch) {
          capsule.halfHeight = ((this.crouchHeight * 0.5) - this.radius);
          rigidBody.transform[10] -= ((this.height - this.crouchHeight) * 0.5);
        } else {
          capsule.halfHeight = ((this.height * 0.5) - this.radius);
          rigidBody.transform[10] += ((this.height - this.crouchHeight) * 0.5);
        }
        if (rigidBody._world) {
          rigidBody._world.wakeBody(rigidBody);
        }
      }
    }
    bool get dead => _private.dead;
    void set dead(bool dead) {
      var pc = this._private;
      if (pc.dead != dead) {
        var rigidBody = pc.rigidBody._private;
        var capsule = rigidBody.shape;

        pc.dead = dead;
        if (dead)
        {
          capsule.halfHeight = 0;
          rigidBody.transform[10] -= ((this.height - this.radius) * 0.5);
        }
        else
        {
          capsule.halfHeight = ((this.height * 0.5) - this.radius);
          rigidBody.transform[10] += ((this.height - this.radius) * 0.5);
        }

        if (rigidBody._world)
        {
          rigidBody._world.wakeBody(rigidBody);
        }
      }
    }
    double maxJumpHeight;
    dynamic userData;

    WebGLPhysicsPrivateCharacter _private;

    double get height => _height;
    double _height;
    double get radius => _radius;
    double _radius;
    double get stepHeight => _stepHeight;
    double _stepHeight;
    double get crouchHeight => _crouchHeight;
    double _crouchHeight;

    bool get onGround {
      var pc = this._private;
      var rigidBody = pc.rigidBody._private;

      if (rigidBody._world) {
        var pos = rigidBody.transform;
        var start = pc.start;
        var end   = pc.end;

        start[9]  = pos[9];
        start[10] = pos[10];
        start[11] = pos[11];

        end[9]  = pos[9];
        end[10] = (pos[10] - (this.stepHeight * 0.5));
        end[11] = pos[11];

        var result = rigidBody._world.convexSweepTest({
          shape: rigidBody.shape._public,
          from: start,
          to: end,
          group: WebGLPhysicsDevice.prototype.FILTER_CHARACTER},
          this.onGroundConvexCallback);
        return (result != null);
      } else {
        return false;
      }
    }


    Float32List get position {
      var rigidBody = this._private.rigidBody;
      return MathHelper.m43Pos(rigidBody._private.transform);
    }
    void set position(Float32List position) {
      var rigidBody = this._private.rigidBody;
      var transform = rigidBody._private.transform;
      transform[9]  = position[0];
      transform[10] = position[1];
      transform[11] = position[2];
      rigidBody.transform = rigidBody._private.transform; //invoke setter.
      rigidBody.active = true;
    }

    Float32List get velocity {
      var rigidBody = this._private.rigidBody;
      return rigidBody.linearVelocity;
    }
    void set velocity(Float32List velocity) {
      var rigidBody = this._private.rigidBody;
      rigidBody.linearVelocity = velocity;
      rigidBody.active = true;
    }


    int _group, _mask;
    int get group => _group;
    int get mask => _mask;


    Float32List start; // m43
    Float32List end;   // m43
    WebGLPhysicsRigidBody rigidBody;

    void jump() {
        var pc = this._private;
        var rigidBody = pc.rigidBody._private;
        var world = rigidBody._world;
        if (world) {
            rigidBody.velocity[1] = Math.sqrt(-2 * (this.maxJumpHeight - this.stepHeight) * world.gravity[1]);
            rigidBody.transform[10] += this.stepHeight;
            world.wakeBody(rigidBody);
        }
    }

    void calculateExtents(Float32List extents) {
        _private.rigidBody.calculateExtents(extents);
    }

    void calculateTransform(Float32List transform, Float32List origin) {
        _private.rigidBody.calculateTransform(transform, origin);
    }

    WebGLPhysicsCharacter({dynamic userData, int group: WebGLPhysicsDevice.FILTER_CHARACTER, int mask: WebGLPhysicsDevice.FILTER_ALL, double height, double radius, double crouchHeight, double stepHeight: 0.35, double maxJumpHeight: 1.0}) {
        var c = this;
        var pc = new WebGLPhysicsPrivateCharacter();
        c._private = pc;

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
        var capsule = new WebGLPhysicsCapsuleShape({
          radius : c.radius,
          height : (2.0 * ((c.height * 0.5) - c.radius)),
          margin : 0.0
        });

        var rigidBody = new WebGLPhysicsRigidBody({
            shape : capsule,
            mass : params.mass,
            transform : params.transform,
            linearVelocity : params.velocity,
            group : group,
            mask : mask,
            friction : params.friction,
            restitution : params.restitution,
            linearDamping : params.linearDamping,
            angularDamping : params.angularDamping,
            fixedRotation : true
        });

        // private (But internals like dynamics world need access through this object).
        pc.rigidBody = rigidBody;

        // Back reference to this public character, so that rayTests and
        // convexSweeps can in the case of intersecting a rigid body that
        // represents a character, return the character instead!

        // TODO: Here a WebGLPhysicsCharacter is assign to a property that
        // should be a WebGLPhysicsCollisionObject.  Can PhysicsCharacter
        // inherit from CollisionObject?

        rigidBody._private._public = /*<WebGLPhysicsCollisionObject><any>*/c;
    }
}

class WebGLPhysicsPrivateCharacter {
    static const int version = 1;

    bool crouch;
    bool dead;
    Float32List start; // m43
    Float32List end;   // m43
    WebGLPhysicsRigidBody rigidBody;

    WebGLPhysicsPrivateCharacter() {
      // Initialise all properties this object will ever hold.

      // Value of read/write property.
      crouch = false;
      dead = false;

      // Matrices re-used in all calls to onGround getter.
      start = MathHelper.m43BuildIdentity();
      end   = MathHelper.m43BuildIdentity();

      // Reference to created RigidBody representing Character.
      rigidBody = null;
    }

    bool onGroundConvexCallback(hitResult)
    {
        // Less than cosine of 15 degrees.
        return hitResult.hitNormal[1] >= 0.26;
    }
}
