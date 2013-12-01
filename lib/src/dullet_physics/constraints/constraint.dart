part of dullet_physics;



//
// WebGLPhysicsConstraint
//
class WebGLPhysicsConstraint /*implements PhysicsConstraint*/
{
  static const int version = 1;

  WebGLPhysicsIsland _island;
  double _wakeTimeStamp;

  // PhysicsConstraint
  WebGLPhysicsPrivateBody bodyA;
  WebGLPhysicsPrivateBody bodyB;
  Float32List transformA;// : any; // m43
  Float32List transformB;// : any; // m43
  //String type;

  WebGLPhysicsWorld _world;
  bool _active;
  WebGLPhysicsConstraint _islandRoot;
  int _islandRank;
  dynamic userData;

  //var _private;

  preStep(double timeStepRatio, double timeStep) {

  }

  applyCachedImpulses() {
  }

  computeAndApplyImpulses() {
  }

  WebGLPhysicsConstraint({
  WebGLPhysicsPrivateBody bodyA, 
  WebGLPhysicsPrivateBody bodyB, 
  bool active}
  ) {
    var s = this;

    s._world = null;
    s.userData = null;

    //webGLPhysicsClone(s, params);
    //s.type = type;


    //initConstraintProperties
    var pc = this;
    pc._world = null;

    pc.bodyA = bodyA;
    pc.bodyB = bodyB;
    pc._active = active;


  }
  bool get active => this._active;
  void set active(bool bActive) {
    var pc = this;
    if (bActive == pc._active) {
      // If already active, and in a world then allow re-setting to true
      // to update wakeTimeStamp.
      if (pc._world != null && bActive) {
        pc._wakeTimeStamp = pc._world._timeStamp;
      }
    } else {
      if (pc._world != null) {
        // If in a world, and not already active then wake the constraint.
        if (bActive) {
          pc._world._wakeConstraint(pc);
        } else { // Otherwise force constraint to go to sleep.
          /*var list = pc._world.activeConstraints;
          list[list.indexOf(pc)] = list[list.length - 1];
          list.pop();*/
          _world._activeConstraints.remove(this);
          pc._active = false;
        }
      } else {
        pc._active = bActive;
      }
    }
  }
}
/*
// Decorate constraint with getter/setters for bodyA/bodyB
// And deal with construction logic common to all constraints
var initConstraintProperties = function initConstraintPropertiesFn(c, params)
{
    c.userData = params.userData;

    var pc = c._private;
    pc.world = null;

    // read only, no getter required.
    pc.bodyA = params.bodyA._private;
    Object.defineProperty(c, "bodyA", {
        value : params.bodyA,
        enumerable : true
    });

    // read only, no getter required.
    pc.bodyB = params.bodyB ? params.bodyB._private : null;
    Object.defineProperty(c, "bodyB", {
        value : params.bodyB,
        enumerable : true
    });

    // read/write with side effects
    pc.active = (params.active !== undefined) ? params.active : true;
    Object.defineProperty(c, "active", {
        get : function constraintGetActive()
        {
            return this._private.active;
        },
        set : function constraintSetActive(active)
        {
         ....
        },
        enumerable : true
    });
}
*/
