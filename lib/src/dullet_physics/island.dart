part of dullet_physics;




//
// WebGLPhysicsIsland
//
class WebGLPhysicsIsland {
    static const version = 1;

    List<WebGLPhysicsPrivateBody> _bodies;//List<WebGLPhysicsRigidBody> bodies;
    List<WebGLPhysicsPrivateBody> _constraints;//List<WebGLPhysicsConstraint> constraints;
    double _wakeTimeStamp;
    bool _active;

    WebGLPhysicsIsland() {
      // Initialise all properties of islands
      // which will ever be used.

      // Set of rigid bodies in island
      _bodies = <WebGLPhysicsPrivateBody>[];//bodies = <WebGLPhysicsRigidBody>[];

      // Set of constraints in island
      _constraints = <WebGLPhysicsPrivateBody>[];//<WebGLPhysicsConstraint>[];

      // Local max wakeTimeStamp for island
      this._wakeTimeStamp = 0.0;

      // Active state of island (compared to sleeping)
      this._active = false;

      //return this;
    }

    // Island objects are object pooled due to being frequently created
    // and destroyed.
    //
    // Island are thus instead allocated and deallocated with no create
    // method.
    //
    // Object pool for islands

    static List<WebGLPhysicsIsland> islandPool = [];
    //static int islandPoolSize = 0;

    static WebGLPhysicsIsland allocate() {
        var island;
        int islandPoolSize = islandPool.length;
        if (islandPoolSize == 0) {
            island = new WebGLPhysicsIsland();
        } else {
            island = islandPool.removeLast();
            //islandPoolSize -= 1;
        }

        return island;
    }

    static void deallocate(WebGLPhysicsIsland island) {
      islandPool.add(island);
      // Make sure to reset local max wakeTimeStamp back to 0.
      island._wakeTimeStamp = 0.0;
    }
}

//
// WebGLPhysicsTOIEvent
//
class WebGLPhysicsTOIEvent {
    static const int version = 1;

    WebGLPhysicsPrivateBody objectA, objectB;
    //WebGLPhysicsCollisionObject objectA;
    //WebGLPhysicsCollisionObject objectB;
    WebGLPhysicsShape shapeA;
    WebGLPhysicsShape shapeB;

    Vector3 closestA; // v3
    Vector3 closestB; // v3
    Vector3 axis; // v3
    double distance;
    double toi;
    bool frozenA;
    bool frozenB;
    bool concave;

    WebGLPhysicsTOIEvent()
    {
        // Initialise all properties of TOI Event
        // which will ever be used.
        //
        // This object is made to dual as a cache in contactPairTest.

        // Objects TOI Event relates to.
        this.objectA = null;
        this.objectB = null;

        // Shapes TOI Event relates to.
        // Precondition: object#.shape = shape#
        this.shapeA = null;
        this.shapeB = null;

        // Closest points on shapes forming the contact point.
        this.closestA = new Vector3.zero();
        this.closestB = new Vector3.zero();

        // Seperating axis / MTV axis forming contact normal.
        this.axis = new Vector3.zero();

        // Penetration distance for contact of TOI event.
        this.distance = 0.0;

        // Time of impact for this event.
        this.toi = 0.0;

        // Cache defining the frozen state of objects during continuous collision detection.
        // Used to invalidate TOI Event when an object's sweepFrozen differs.
        this.frozenA = false;
        this.frozenB = false;

        // Marks this event as corresponding to a concave triangle mesh.
        // This value is passed to insertContact to prevent culling contacts
        // based on normals.
        this.concave = false;
    }

    //
    // TOI Events are object pooled due to being frequently created
    // and destroyed. TOI Events are thus instead allocated and
    // deallocated with no create method.
    //
    // Object pool for TOI Events

    static final List<WebGLPhysicsTOIEvent> eventPool = <WebGLPhysicsTOIEvent>[];

    static WebGLPhysicsTOIEvent allocate() {
        var toi;
        if (eventPool.isEmpty) {
            toi = new WebGLPhysicsTOIEvent();
        } else {
            toi = eventPool.removeLast();
        }

        return toi;
    }

    static void deallocate(WebGLPhysicsTOIEvent toi) {
      eventPool.add(toi);
      // Ensure that if this is a concave TOI Event, that we also
      // deallocate the related TriangleShape that is generated.
      if (toi.concave){
        WebGLPhysicsTriangleShape.deallocate(toi.shapeB as WebGLPhysicsTriangleShape);
        toi.concave = false;
      }

      // Ensure that object references are set to null to permit GC
      // even if this is in the object pool.
      toi.objectA = null;
      toi.objectB = null;
      toi.shapeA = null;
      toi.shapeB = null;
    }
}




