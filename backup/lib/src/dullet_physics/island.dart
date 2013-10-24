part of dullet_physics;




//
// WebGLPhysicsIsland
//
class WebGLPhysicsIsland {
    static const version = 1;

    List<WebGLPhysicsPrivateBody> bodies;//List<WebGLPhysicsRigidBody> bodies;
    List<WebGLPhysicsPrivateBody> constraints;//List<WebGLPhysicsConstraint> constraints;
    double wakeTimeStamp;
    bool active;

    DWebGLPhysicsIsland() {
      // Initialise all properties of islands
      // which will ever be used.

      // Set of rigid bodies in island
      bodies = <WebGLPhysicsPrivateBody>[];//bodies = <WebGLPhysicsRigidBody>[];

      // Set of constraints in island
      this.constraints = <WebGLPhysicsPrivateBody>[];//<WebGLPhysicsConstraint>[];

      // Local max wakeTimeStamp for island
      this.wakeTimeStamp = 0.0;

      // Active state of island (compared to sleeping)
      this.active = false;

      return this;
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
      island.wakeTimeStamp = 0.0;
    }
}

//
// WebGLPhysicsTriangleShape
//
class WebGLPhysicsTriangleShape
{
    static const version = 1;

    final String type = "TRIANGLE_MESH_TRIANGLE"; // prototype
    int index;
    double collisionRadius;
    WebGLPhysicsPrivateTriangleArray triangleArray;

    WebGLPhysicsTriangleShape()
    {
        // Initialise all properties of Triangle shape
        // which will ever be used.

        // Index into parent WebGLTriangleArray::triangles list.
        this.index = 0;

        // Collision radius in collision algorithms, this is taken from parent mesh shape.
        this.collisionRadius = 0.0;

        // The parent TriangleArray.
        this.triangleArray = null;

    }

    localSupportWithoutMargin(Float32List vec, Float32List dst) {
        var vec0 = vec[0];
        var vec1 = vec[1];
        var vec2 = vec[2];

        var triangles = this.triangleArray.triangles;
        var triangle = this.index;

        var v00 = triangles[triangle + 3];
        var v01 = triangles[triangle + 4];
        var v02 = triangles[triangle + 5];
        var u0 = triangles[triangle + 6];
        var u1 = triangles[triangle + 7];
        var u2 = triangles[triangle + 8];
        var v0 = triangles[triangle + 9];
        var v1 = triangles[triangle + 10];
        var v2 = triangles[triangle + 11];

        var dotu = ((vec0 * u0) + (vec1 * u1) + (vec2 * u2));
        var dotv = ((vec0 * v0) + (vec1 * v1) + (vec2 * v2));

        if (dotu <= 0 && dotv <= 0) {
            dst[0] = v00;
            dst[1] = v01;
            dst[2] = v02;
        } else if (dotu >= dotv) {
            dst[0] = (v00 + u0);
            dst[1] = (v01 + u1);
            dst[2] = (v02 + u2);
        } else {
            dst[0] = (v00 + v0);
            dst[1] = (v01 + v1);
            dst[2] = (v02 + v2);
        }
    }

    //
    // Triangle shapes are used for collision detection with triangles
    // of a triangle mesh.  In most cases there is one persistant
    // object used. In the case of continuous collisions we need to
    // reuse many of these objects, so an object pool is used with
    // allocate and deallocate methods instead of a create method.
    //
    // Object pool for Triangles

    static final List<WebGLPhysicsTriangleShape> trianglePool = <WebGLPhysicsTriangleShape>[];

    static WebGLPhysicsTriangleShape allocate()
    {
        var triangle;
        if (trianglePool.isEmpty) {
            triangle = new WebGLPhysicsTriangleShape();
        } else {
            triangle = trianglePool.removeLast();
        }

        return triangle;
    }

    static void  deallocate(WebGLPhysicsTriangleShape triangle) {
      trianglePool.add(triangle);
      // Ensure reference is null'ed so that an object pooled Triangle Shape
      // cannot prevent the TriangleArray from being GC'ed.

      triangle.triangleArray = null;
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
        if (toi.concave)
        {
            WebGLPhysicsTriangleShape.deallocate(toi.shapeB);
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




