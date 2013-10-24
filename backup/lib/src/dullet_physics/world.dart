part of dullet_physics;

class TurbulenzEngine {
  static double getTime() => 0.0;
}


class RayTestParameters {
  int group, mask, exclude;
  Vector3 from, to;
}

//
// WebGLPhysicsWorld
//
class WebGLPhysicsWorld /*implements PhysicsWorld*/ {
    static const version = 1;

    // PhysicsWorld
    Vector3 get gravity => _private.gravity.clone();
    int get maxSubSteps => _private.maxSubSteps;
    double get fixedTimeStep => _private.fixedTimeStep;
    double get maxGiveUpTimeStep => _private.maxGiveUpTimeStep;
    double get minimumTimeStep => _private.variableMinStep;
    double get maximumTimeStep => _private.variableMaxStep;
    dynamic get performanceData => _private.performanceData;

    double timeStamp;

    WebGLPrivatePhysicsWorld _private;

    List<WebGLPhysicsCollisionObject> contactCallbackObjects;

    void update() {
        this._private.update();
    }

    RayHit rayTest(RayTestParameters ray) {
        return this._private.rayTest(ray);
    }

    RayHit convexSweepTest(/*ConvexSweepTestParameters params*/{
      WebGLPhysicsShape shape,
      Matrix43 from,
      Matrix43 to,
      int group: WebGLPhysicsDevice.FILTER_DYNAMIC,
      int mask: WebGLPhysicsDevice.FILTER_ALL,
      int exclude, callback}) {
        return this._private.convexSweepTest(
            shape: shape,
            from: from,
            to: to,
            group: group,
            mask: mask,
            callback: callback);
    }

    addCollisionObject(WebGLPhysicsCollisionObject collisionObject) {
        return this._private.addBody( collisionObject._private);
    }

    removeCollisionObject(WebGLPhysicsCollisionObject collisionObject) {
        return this._private.removeBody( collisionObject._private);
    }

    addRigidBody(WebGLPhysicsRigidBody rigidBody) {
        return this._private.addBody( rigidBody._private);
    }

    removeRigidBody(WebGLPhysicsRigidBody rigidBody) {
        return this._private.removeBody( rigidBody._private);
    }

    addConstraint(WebGLPhysicsConstraint constraint)
    {
        return this._private.addConstraint( constraint._private);
    }

    removeConstraint(WebGLPhysicsConstraint constraint)
    {
        return this._private.removeConstraint(
            constraint._private);
    }

    addCharacter(WebGLPhysicsCharacter character)
    {
        return this._private.addBody(
            character._private.rigidBody._private);
    }

    removeCharacter(WebGLPhysicsCharacter character)
    {
        return this._private.removeBody( character._private.rigidBody._private);
    }

    wakeBody(WebGLPhysicsPrivateBody body)
    {
        this._private.wakeBody(body);
    }

    flush() {
        this._private.flush();
    }

    WebGLPhysicsWorld({Vector3 gravity,
          int maxSubSteps: 10,
          double fixedTimeStep: 1.0/60.0,
          double variableMinStep: 1.0/70.0,
          double variableMaxStep: 1.0/50.0,
          bool variableTimeSteps: false,
          double maxGiveUpTimeStep: 1.0 / 20.0
          }) {
      if(gravity == null) {
        gravity = new Vector3(0.0,-10.0,0.0);
      }
        var rets = this;
        var s = new WebGLPrivatePhysicsWorld();
        rets._private = s;
        s._public = rets;

        s.gravity = gravity;
        s.maxSubSteps = maxSubSteps;//(params.maxSubSteps != null) ? params.maxSubSteps : 10;

        s.fixedTimeStep = fixedTimeStep;//(params.fixedTimeStep != null) ? params.fixedTimeStep : (1 / 60);

        s.variableMinStep = variableMinStep;//(params.minimumTimeStep != null) ? params.minimumTimeStep : (1 / 70);
        s.variableMaxStep = variableMaxStep;//(params.maximumTimeStep != null) ? params.maximumTimeStep : (1 / 50);

        s.variableStep = variableTimeSteps;//(params.variableTimeSteps != null) ? params.variableTimeSteps : false;

        s.maxGiveUpTimeStep = maxGiveUpTimeStep;//(params.maxGiveUpTimeStep != null) ? params.maxGiveUpTimeStep : 1 / 20;

        s.staticSpatialMap = new AabbTree(true);
        s.dynamicSpatialMap = new AabbTree();

        s.collisionObjects = [];
        s.rigidBodies = [];
        s.constraints = [];
        s.kinematicBodies = [];

        // List of active arbiters between shapes.
        s.activeArbiters = [];

        // List of active rigid bodies and constraints.
        s.activeBodies = [];
        s.activeKinematics = [];
        s.activeConstraints = [];

        s.persistantObjectsList = [];
        s.persistantObjectsList2 = [];
        s.persistantTrianglesList = [];
        s.persistantTOIEventList = [];

        s.timeStamp = 0;

        // timing information
        s.performanceData = new WebGLPhysicsPerformance();/*{
            discrete             : 0,
            sleepComputation     : 0,
            prestepContacts      : 0,
            prestepConstraints   : 0,
            integrateVelocities  : 0,
            warmstartContacts    : 0,
            warmstartConstraints : 0,
            physicsIterations    : 0,
            integratePositions   : 0,
            continuous           : 0
        };*/

        // Extents used throughout all calls to syncBody
        s.syncExtents = new Float32List(6);

        // Array for all the objects we need to call for contact callbacks
        s.contactCallbackObjects = [];

        // Array for all the removed arbiters
        s.contactCallbackRemovedArbiters = [];

        //return rets;
    }
}

class WebGLPhysicsPerformance {
  int discrete = 0;
  int sleepComputation = 0;
  int prestepContacts = 0;
  int prestepConstraints = 0;
  int integrateVelocities = 0;
  int warmstartContacts = 0;
  int warmstartConstraints = 0;
  int physicsIterations = 0;
  int integratePositions = 0;
  int continuous = 0;
}

class WebGLPhysicsNarrowCache extends WebGLPhysicsTOIEvent {
    //Vector3 axis = new Vector3.zero(); // v3
    //WebGLPhysicsShape shapeA; // TODO: what is this
    //WebGLPhysicsShape shapeB; // TODO: what is this
    //Vector3 closestA = new Vector3.zero(); // v3
    //Vector3 closestB = new Vector3.zero(); // v3
    WebGLPhysicsNarrowCache() {
      axis = new Vector3.zero(); // v3
      closestA = new Vector3.zero(); // v3
      closestB = new Vector3.zero(); // v3
    }
    WebGLPhysicsNarrowCache.fromData(
        Vector3 axis,
        WebGLPhysicsShape shapeA,
        WebGLPhysicsShape shapeB,
        Vector3 closestA,
        Vector3 closestB) {
      this.axis = axis;
      this.shapeA = shapeA;
      this.shapeB = shapeB;
      this.closestA = closestA;
      this.closestB = closestB;
    }

}
class FakeBody {
  WebGLPhysicsShape shape;
  Matrix43 transform = new Matrix43();
  Matrix43 startTransform = new Matrix43();
}

class WebGLPrivatePhysicsWorld
{
    static const version = 1;
    WebGLPhysicsWorld _public;

    //_public                        : WebGLPhysicsWorld;

    Vector3 gravity;
    int maxSubSteps;
    double fixedTimeStep;
    double variableMinStep;
    double variableMaxStep;
    bool variableStep;
    double maxGiveUpTimeStep;
    AabbTree staticSpatialMap;
    AabbTree dynamicSpatialMap;
    List<WebGLPhysicsCollisionObject> collisionObjects;
    List<WebGLPhysicsRigidBody> rigidBodies;
    List<WebGLPhysicsConstraint> constraints;
    List<WebGLPhysicsCollisionObject> kinematicBodies;

    final Vector3 planeAxis = new Vector3.zero(); // v3
    final Vector3 planeSA = new Vector3.zero(); // v3
    final Vector3 planeSB = new Vector3.zero(); // v3

    bool midStep;

    WebGLPhysicsTriangleShape narrowTriangle;
    WebGLPhysicsNarrowCache narrowCache;
    WebGLPhysicsNarrowCache narrowCache2;
    FakeBody narrowFakeBody;
    /*narrowFakeBody: {
        transform : any; // v3
        shape : any; // TODO: what is this
    };*/

    Matrix43 narrowTransform; // m43
    Float32List narrowExtents;

    /*continuousFakeBody: {
        shape : any; // TODO: type
        transform : any; // m43
        startTransform : any; // m43
    };*/
    FakeBody continuousFakeBody = new FakeBody();
    Matrix43 continuousInvTransform; // m43
    Float32List continuousExtents;

    List<WebGLPhysicsArbiter> activeArbiters;

    List<WebGLPhysicsPrivateBody> activeBodies;
    List<WebGLPhysicsPrivateBody> activeKinematics;
    List<WebGLPhysicsConstraint> activeConstraints;

    List<WebGLPhysicsPrivateBody> persistantObjectsList;//List<WebGLPhysicsRigidBody> persistantObjectsList;
    List<WebGLPhysicsPrivateBody> persistantObjectsList2;//List<WebGLPhysicsRigidBody> persistantObjectsList2;
    List persistantTrianglesList; // TODO: type?
    List<WebGLPhysicsTOIEvent> persistantTOIEventList;

    double prevTimeStep;

    double prevTimeStamp;
    double timeStamp;

    WebGLGJKContactSolver contactGJK;
    WebGLContactEPA contactEPA;

    // TODO: reuse narrocache type?
    /*
    sweepCache: {
        axis : any; // v3
        shapeA : any; // TODO: type
        shapeB : any; // TODO: type
        closestA : any; // v3
        closestB : any; // v3
    };*/
    WebGLPhysicsTriangleShape sweepTriangle = WebGLPhysicsTriangleShape.allocate();
    Vector3 sweepDelta = new Vector3.zero(); // v3
    Aabb3 sweepFromExtents = new Aabb3();
    Aabb3 sweepToExtents = new Aabb3();
    Aabb3 sweepExtents = new Aabb3();

    WebGLPhysicsNarrowCache sweepCache = new WebGLPhysicsNarrowCache();
    FakeBody sweepFakeBody = new FakeBody();

    /*
    sweepFakeBody: {
        shape : any;
        transform : any;
    };*/
    Matrix43 sweepTransform = new Matrix43.identity(); // m43
    Matrix43 sweepTransform2 = new Matrix43.identity(); // m43

    // timing information
    var performanceData; /*                : {
        discrete                   : number;
        sleepComputation           : number;
        prestepContacts            : number;
        prestepConstraints         : number;
        integrateVelocities        : number;
        warmstartContacts          : number;
        warmstartConstraints       : number;
        physicsIterations          : number;
        integratePositions         : number;
        continuous                 : number;
    };*/

    Float32List syncExtents;
    List<WebGLPhysicsPrivateBody> contactCallbackObjects;
    List<WebGLPhysicsArbiter> contactCallbackRemovedArbiters;

    Vector3 m43InverseOrthonormalTransformVector(Matrix43 m, Vector3 v, [Vector3 dst]) {
        if (dst == null) {
            dst = new Vector3.zero();
        }
        var v0 = v.storage[0];
        var v1 = v.storage[1];
        var v2 = v.storage[2];
        dst.storage[0] = (m.storage[0] * v0 + m.storage[1] * v1 + m.storage[2] * v2);
        dst.storage[1] = (m.storage[3] * v0 + m.storage[4] * v1 + m.storage[5] * v2);
        dst.storage[2] = (m.storage[6] * v0 + m.storage[7] * v1 + m.storage[8] * v2);
        return dst;
    }

    Vector3 m43InverseOrthonormalTransformPoint(Matrix43 m, Vector3 v, [Vector3 dst]) {
      if (dst == null) {
          dst = new Vector3.zero();
      }
      var v0 = v.storage[0] - m.storage[9];
      var v1 = v.storage[1] - m.storage[10];
      var v2 = v.storage[2] - m.storage[11];
      dst.storage[0] = (m.storage[0] * v0 + m.storage[1] * v1 + m.storage[2] * v2);
      dst.storage[1] = (m.storage[3] * v0 + m.storage[4] * v1 + m.storage[5] * v2);
      dst.storage[2] = (m.storage[6] * v0 + m.storage[7] * v1 + m.storage[8] * v2);
      return dst;
    }

    // Determine if shape intersects the plane containing triangle
    // number 'index' in triangle array With given shape and triangle
    // transforms.
    bool trianglePlaneDiscard(WebGLPhysicsShape shape, Matrix43 xform, triangleArray, int index, Matrix43 txform)
    {
        /*if (this.planeAxis == null) {
            this.planeAxis = new Vector3.zero();
            this.planeSA = new Vector3.zero();
            this.planeSB = new Vector3.zero();
        }*/
        var axis = this.planeAxis;
        var supportA = this.planeSA;
        var supportB = this.planeSB;

        var triangles = triangleArray.triangles;
        // local plane normal and distance.
        var n0 = triangles[index];
        var n1 = triangles[index + 1];
        var n2 = triangles[index + 2];
        var nd = triangles[index + 16];

        var A0 = txform.storage[0];
        var A1 = txform.storage[1];
        var A2 = txform.storage[2];
        var A3 = txform.storage[3];
        var A4 = txform.storage[4];
        var A5 = txform.storage[5];
        var A6 = txform.storage[6];
        var A7 = txform.storage[7];
        var A8 = txform.storage[8];
        var A9 = txform.storage[9];
        var A10 = txform.storage[10];
        var A11 = txform.storage[11];

        // transform plane normal into world space.
        var w0 = (n0 * A0) + (n1 * A3) + (n2 * A6);
        var w1 = (n0 * A1) + (n1 * A4) + (n2 * A7);
        var w2 = (n0 * A2) + (n1 * A5) + (n2 * A8);

        A0 = xform.storage[0];
        A1 = xform.storage[1];
        A2 = xform.storage[2];
        A3 = xform.storage[3];
        A4 = xform.storage[4];
        A5 = xform.storage[5];
        A6 = xform.storage[6];
        A7 = xform.storage[7];
        A8 = xform.storage[8];
        A9 -= xform.storage[9];
        A10 -= xform.storage[10];
        A11 -= xform.storage[11];

        // transform plane into shape local space.
        n0 = (A0 * w0) + (A1 * w1) + (A2 * w2);
        n1 = (A3 * w0) + (A4 * w1) + (A5 * w2);
        n2 = (A6 * w0) + (A7 * w1) + (A8 * w2);
        nd += (w0 * A9) + (w1 * A10) + (w2 * A11);

        // find maximum and minimal support points on shape.
        axis.storage[0] = n0;
        axis.storage[1] = n1;
        axis.storage[2] = n2;
        shape.localSupportWithoutMargin(axis, supportA);

        axis.storage[0] = -n0;
        axis.storage[1] = -n1;
        axis.storage[2] = -n2;
        shape.localSupportWithoutMargin(axis, supportB);

        // Find distance from plane for each support.
        var dot1 = (supportA.storage[0] * n0) + (supportA.storage[1] * n1) + (supportA.storage[2] * n2) - nd;
        var dot2 = (supportB.storage[0] * n0) + (supportB.storage[1] * n1) + (supportB.storage[2] * n2) - nd;

        // If supports are on opposite side of plane, primitive definately
        // intersects plane
        if ((dot1 * dot2) <= 0)
        {
            return false;
        }

        // Choose closest support to plane for distance computation
        // with margins.
        var seperation;
        if ((dot1 * dot1) < (dot2 * dot2))
        {
            seperation = dot1;
        }
        else
        {
            seperation = dot2;
        }

        if ((seperation < 0) != ((dot1 * dot2) < 0))
        {
            seperation = -seperation;
        }

        return (seperation - shape.collisionRadius) > 0;
    }

    // Determine if pair of objects is permitted to collide.
    bool filtered(WebGLPhysicsPrivateBody objectA, WebGLPhysicsPrivateBody objectB) {
        if (objectA == objectB) {
            return true;
        }
        if ((objectA.collisionObject || objectA.kinematic) && (objectB.collisionObject || objectB.kinematic)) {
            return true;
        }

        /*jshint bitwise: false*/
        if ((objectA.mask & objectB.group) == 0 ||
            (objectB.mask & objectA.group) == 0) {
            return true;
        }
        /*jshint bitwise: true*/
        return false;
    }

    // perform narrow phase collision detection between shapes A and B
    // owned by respective objects objectA, objectB (objectA.shape ==
    // shapeA etc)
    void narrowPhase(WebGLPhysicsShape shapeA, WebGLPhysicsShape shapeB, WebGLPhysicsPrivateBody objectA, WebGLPhysicsPrivateBody objectB)
    {
        // Objects reused in all narrowPhase calls.
        if (this.narrowTriangle == null)
        {
            // Fake triangle shape for TRIANGLE_MESH collisions.
            this.narrowTriangle = WebGLPhysicsTriangleShape.allocate();

            // contactPairTest cache object.
            this.narrowCache = new WebGLPhysicsNarrowCache.fromData(
                new Vector3(1.0, 0.0, 0.0),
                null,
                null,
                new Vector3.zero(),
                new Vector3.zero()
            );

            // contactPairTest cache object used in TRIANGLE_MESH
            // as shapeA/shapeB are not the same as above.
            this.narrowCache2 = new WebGLPhysicsNarrowCache.fromData(
                this.narrowCache.axis, //reference!
                null,
                null,
                this.narrowCache.closestA, //reference!
                this.narrowCache.closestB //reference!
            );

            // Fake body used for TRIANGLE_MESH to compute local extents of
            // A shape in triangle mesh local-coordinates.
            this.narrowFakeBody.transform = new Matrix43.identity();
            this.narrowFakeBody.shape = null;
            /*this.narrowFakeBody = {
                transform : VMath.m43BuildIdentity(),
                shape : null
            };*/

            this.narrowTransform = new Matrix43.identity();
            this.narrowExtents = new Float32List(6);
        }

        // Find existing arbiter for shape pair.
        // Iterating the smaller list of either object.
        var arb = null;
        var arbitersA = objectA.arbiters;
        var arbitersB = objectB.arbiters;
        var arbiters = (arbitersA.length <= arbitersB.length) ? arbitersA : arbitersB;

        var i = 0;
        var numArbiters = arbiters.length;
        for (i = 0; i < numArbiters; i += 1)
        {
            var carb = arbiters[i];
            if (carb.shapeA == shapeA && carb.shapeB == shapeB &&
                carb.objectA == objectA && carb.objectB == objectB)
            {
                arb = carb;
                break;
            }
        }

        if (arb != null && arb.skipDiscreteCollisions)
        {
            arb.skipDiscreteCollisions = false;
            return;
        }

        // If arbiter does not already exist, create a new one.
        var fresh = (arb == null);
        if (fresh)
        {
            arb = WebGLPhysicsArbiter.allocate(shapeA, shapeB, objectA, objectB);
        }

        var cache = this.narrowCache;
        cache.shapeA = shapeA;
        cache.shapeB = shapeB;

        // 'warm-start' contact solver with a guess direction of MTV
        if (arb.contacts.length != 0)
        {
            //var basis = arb.contacts[0].basis;
            var data = arb.contacts[0];
            // VMath.v3Copy(c.normal, cache.axis);
            cache.axis[0] = data[12];
            cache.axis[1] = data[13];
            cache.axis[2] = data[14];
        }

        var contact;
        var collided = false;

        //
        // Special case for triangle meshes.
        //
        if (shapeA.type == "TRIANGLE_MESH" || shapeB.type == "TRIANGLE_MESH")
        {
            var meshShape, otherShape;
            var meshXForm, otherXForm;
            var triangle = this.narrowTriangle;
            var cache2 = this.narrowCache2;

            if (shapeA.type == "TRIANGLE_MESH") {
                meshShape = shapeA;
                meshXForm = objectA.transform;
                otherShape = shapeB;
                otherXForm = objectB.transform;
                cache2.shapeA = triangle;
                cache2.shapeB = cache.shapeB;
            } else {
                meshShape = shapeB;
                meshXForm = objectB.transform;
                otherShape = shapeA;
                otherXForm = objectA.transform;
                cache2.shapeA = cache.shapeA;
                cache2.shapeB = triangle;
            }

            var triangleArray = meshShape.triangleArray;
            triangle.triangleArray = triangleArray;
            triangle.collisionRadius = meshShape.collisionRadius;

            var numTriangles;

            if (triangleArray.spatialMap) {
                // determine AABB of non-triangle mesh object in local coordinates
                // of triangle mesh.
                var transform = this.narrowTransform;
                var fakeBody = this.narrowFakeBody;
                var extents = this.narrowExtents;

                //---var itransform = VMath.m43InverseOrthonormal(meshXForm);

                //VMath.m43InverseOrthonormal(meshXForm, transform);
                meshXForm.inverseOrthonormal(transform);

                //VMath.m43Mul(otherXForm, transform, fakeBody.transform);
                otherXForm.multiply(transform, fakeBody.transform);
                fakeBody.shape = otherShape;
                /*WebGLPhysicsPrivateBody.calculateExtents.call*/
                calculateExtents(fakeBody, extents);

                // Find all triangles to test against.
                var triangles = this.persistantTrianglesList;
                numTriangles = triangleArray.spatialMap.getOverlappingNodes(extents, triangles, 0);
                for (i = 0; i < numTriangles; i += 1)
                {
                    var index = triangles[i].index;
                    triangle.index = index;
                    // Prevent GC issues from object being kept in persistent array
                    triangles[i] = null;

                    // Shortcut! Check that shape intersects plane of triangle
                    if (!this.trianglePlaneDiscard(otherShape, otherXForm, triangleArray, index, meshXForm))
                    {
                        contact = this.contactPairTest(cache2, objectA.transform, objectB.transform);
                        if (contact < 0)
                        {
                            arb.insertContact(cache2.closestA, cache2.closestB, cache2.axis, contact, true);
                            collided = true;
                        }
                    }
                }
            } else {
                // If triangle mesh is small, no AABBTree exists
                // And we check all triangles brute-force.
                numTriangles = triangleArray.numTriangles;
                for (i = 0; i < numTriangles; i += 1)
                {
                    triangle.index = (i * WebGLPhysicsPrivateTriangleArray.TRIANGLE_SIZE);
                    if (!this.trianglePlaneDiscard(otherShape, otherXForm, triangleArray, triangle.index, meshXForm))
                    {
                        contact = this.contactPairTest(cache2, objectA.transform, objectB.transform);
                        if (contact < 0)
                        {
                            arb.insertContact(cache2.closestA, cache2.closestB, cache2.axis, contact, true);
                            collided = true;
                        }
                    }
                }
            }
        } else { // Default case, both objects are convex.
            contact = this.contactPairTest(cache, objectA.transform, objectB.transform);
            if (contact < 0) {
                arb.insertContact(cache.closestA, cache.closestB, cache.axis, contact, false);
                collided = true;
            }
        }

        if (collided) {
            // New arbiter, add to object lists.
            if (fresh)
            {
                this.activeArbiters.add(arb);
                arb.active = true;
                objectA.arbiters.add(arb);
                objectB.arbiters.add(arb);
            }

            // Wake objects if sleeping, they've just collided!
            if (objectA.permitSleep && !objectA.active)
            {
                this.wakeBody(objectA);
            }
            if (objectB.permitSleep && !objectB.active)
            {
                this.wakeBody(objectB);
            }

            // If arbiter was sleeping, then waking it
            if (!arb.active)
            {
                arb.active = true;
                this.activeArbiters.add(arb);
            }
        }
        else if (fresh)
        {
            // New arbiter, but no collision means we should
            // immediately deallocate for re-use.
            WebGLPhysicsArbiter.deallocate(arb);
        }
    }

    // Compute islands of interaction rigid bodies and constraints
    // And put to sleep those islands that are to be considered
    // stationary.
    void computeSleeping(double timeStep) {
      WebGLPhysicsPrivateBody _find(WebGLPhysicsPrivateBody x) {
            if (x == x.islandRoot)
            {
                return x;
            }

            var root = x;
            var stack = null;
            var next;
            while (root != root.islandRoot)
            {
                next = root.islandRoot;
                root.islandRoot = stack;
                stack = root;
                root = next;
            }

            while (stack != null)
            {
                next = stack.islandRoot;
                stack.islandRoot = root;
                stack = next;
            }
            return root;
        }

        // Implementation of union-find algorithm with union by rank
        // and path compression.
        _unify(WebGLPhysicsPrivateBody x, WebGLPhysicsPrivateBody y) {
            var xr = _find(x);
            var yr = _find(y);
            if (xr != yr) {
                if (xr.islandRank < yr.islandRank) {
                    xr.islandRoot = yr;
                } else if (xr.islandRank > yr.islandRank) {
                    yr.islandRoot = xr;
                } else {
                    yr.islandRoot = xr;
                    xr.islandRank += 1;
                }
            }
        }

        WebGLPhysicsPrivateBody objectA, objectB;

        // Build disjoint set forest
        // based on active arbiters and constraints.
        var arbiters = this.activeArbiters;
        var bodies = this.activeBodies;
        var constraints = this.activeConstraints;

        var n;
        var maxN = arbiters.length;
        for (n = 0; n < maxN; n += 1)
        {
            var arb = arbiters[n];
            objectA = arb.objectA;
            objectB = arb.objectB;
            if (objectA.permitSleep && objectB.permitSleep)
            {
                _unify(objectA, objectB);
            }
        }

        maxN = constraints.length;
        var con;
        for (n = 0; n < maxN; n += 1)
        {
            con = constraints[n];
            objectA = con.bodyA;
            objectB = con.bodyB;
            if (objectA && objectA.permitSleep)
            {
                _unify(objectA, con);
            }
            if (objectB && objectB.permitSleep)
            {
                _unify(objectB, con);
            }
        }

        // Build islands
        List<WebGLPhysicsIsland> islands = <WebGLPhysicsIsland>[];
        var island, body, root;
        while (bodies.length > 0)
        {
            body = bodies.removeLast();
            root = _find(body);
            island = root.island;
            if (!island)
            {
                island = root.island = WebGLPhysicsIsland.allocate();
                islands.add(island);
                island.active = false;
            }

            body.island = island;
            island.bodies.add(body);
            island.active = island.active || body.isActive(/*timeStep*/);
            if (body.wakeTimeStamp > island.wakeTimeStamp)
            {
                island.wakeTimeStamp = body.wakeTimeStamp;
            }
        }

        while (constraints.length > 0)
        {
            con = constraints.removeLast();
            root = _find(con);
            island = root.island;
            if (!island)
            {
                island = root.island = WebGLPhysicsIsland.allocate();
                islands.add(island);
                island.active = true;
            }

            con.island = island;
            island.constraints.add(con);
            if (con.wakeTimeStamp > island.wakeTimeStamp)
            {
                island.wakeTimeStamp = con.wakeTimeStamp;
            }
        }

        // Build new active bodies, destroying islands
        // Keep sleeping islands implicitly through references in sleeping bodies.
        while (islands.length > 0) {
            island = islands.removeLast();
            if (island.active) {
                while (island.bodies.length > 0) {
                    body = island.bodies.removeLast();
                    body.wakeTimeStamp = island.wakeTimeStamp;
                    bodies.add(body);

                    // reset for next iteration of computeSleeping
                    body.islandRoot = body;
                    body.islandRank = 0;
                    body.island = null;
                } while (island.constraints.length > 0) {
                    con = island.constraints.removeLast();
                    con.wakeTimeStamp = island.wakeTimeStamp;
                    constraints.add(con);

                    // reset for next iteration of computeSleeping
                    con.islandRoot = con;
                    con.islandRank = 0;
                    con.island = null;
                }

                WebGLPhysicsIsland.deallocate(island);
            } else {
                maxN = island.bodies.length;
                for (n = 0; n < maxN; n += 1) {
                    body = island.bodies[n];
                    body.velocity[0] = body.velocity[1] = body.velocity[2] = 0;
                    body.velocity[3] = body.velocity[4] = body.velocity[5] = 0;
                    body.active = false;
                    this.syncBody(body);

                    // reset for next iteration of computeSleeping
                    body.islandRoot = body;
                    body.islandRank = 0;
                }

                maxN = island.constraints.length;
                for (n = 0; n < maxN; n += 1) {
                    con = island.constraints[n];
                    con.active = false;

                    // reset for next iteration of computeSleeping
                    con.islandRoot = con;
                    con.islandRank = 0;
                }
            }
        }
    }

    // Wake up a sleeping island.
    void wakeIsland(WebGLPhysicsIsland island)
    {
        while (island.bodies.length > 0)
        {
            var body = island.bodies.removeLast();
            body.wakeTimeStamp = this.timeStamp + (this.midStep ? 0 : 1);
            this.activeBodies.add(body);

            var n;
            var arbiters = body.arbiters;
            var maxN = arbiters.length;
            for (n = 0; n < maxN; n += 1)
            {
                var arb = arbiters[n];
                if (!arb.active)
                {
                    arb.active = true;
                    this.activeArbiters.add(arb);
                }
            }

            body.active = true;
            body.island = null;
            this.syncBody(body);
        }

        while (island.constraints.length > 0)
        {
            var constraint = island.constraints.removeLast();
            constraint.wakeTimeStamp = this.timeStamp + (this.midStep ? 0 : 1);
            this.activeConstraints.add(constraint);

            constraint.active = true;
            constraint.island = null;
        }

        WebGLPhysicsIsland.deallocate(island);
    }

    void wakeRelated(WebGLPhysicsPrivateBody body)
    {
        // Wake any related constraints
        var constraints = body.constraints;
        var n;
        var maxN = constraints.length;
        for (n = 0; n < maxN; n += 1)
        {
            this.wakeConstraint(constraints[n]);
        }

        // Wake any touching bodies
        var arbiters = body.arbiters;
        maxN = arbiters.length;
        for (n = 0; n < maxN; n += 1)
        {
            var arb = arbiters[n];
            if (!arb.active)
            {
                arb.active = true;
                this.activeArbiters.add(arb);
            }

            if (arb.objectA.permitSleep && !arb.objectA.active)
            {
                this.wakeBody(arb.objectA);
            }
            if (arb.objectB.permitSleep && !arb.objectB.active)
            {
                this.wakeBody(arb.objectB);
            }
        }
    }

    // TODO: Should this be taking the private object?  Seems to be
    // given a WebGLPhysicsPrivateBody, but puts it on
    // activeKinematics which

    // Wake up a rigid body.
    void wakeBody(WebGLPhysicsPrivateBody body)
    {
        if (body.collisionObject && !body.kinematic)
        {
            this.wakeRelated(body);
            this.syncBody(body);
        }
        else if (body.kinematic)
        {
            body.delaySleep = true;
            if (!body.active)
            {
                body.active = true;
                this.activeKinematics.add(body);

                this.wakeRelated(body);
                this.syncBody(body);
            }
        }
        else
        {
            body.wakeTimeStamp = this.timeStamp + (this.midStep ? 0 : 1);
            if (!body.active)
            {
                if (!body.island)
                {
                    body.active = true;
                    this.activeBodies.add(body);

                    this.wakeRelated(body);
                    this.syncBody(body);
                }
                else
                {
                    this.wakeIsland(body.island);
                }

                // Synchronise body with broadphase.
                this.syncBody(body);
            }
        }
    }

    // Sync body with broadphase
    void syncBody(WebGLPhysicsPrivateBody body) {
        var extents = this.syncExtents;
        body.calculateExtents(extents);
        if (body.collisionObject && !body.kinematic) {
            this.staticSpatialMap.update(body, extents);
        } else {
            if (body.active) {
                if (!body.previouslyActive) {
                    this.staticSpatialMap.remove(body);
                    this.dynamicSpatialMap.add(body, extents);
                } else {
                    this.dynamicSpatialMap.update(body, extents);
                }
            } else {
                if (body.previouslyActive) {
                    this.dynamicSpatialMap.remove(body);
                    this.staticSpatialMap.add(body, extents);
                } else {
                    this.staticSpatialMap.update(body, extents);
                }
            }

            body.previouslyActive = body.active;
        }
    }

    // Wake up a constraint
    void wakeConstraint(WebGLPhysicsConstraint constraint) {
        constraint.wakeTimeStamp = this.timeStamp + (this.midStep ? 0 : 1);
        if (!constraint.active) {
            if (!constraint.island != null) {
                constraint.active = true;
                this.activeConstraints.add(constraint);

                if (constraint.bodyA != null) {
                    this.wakeBody(constraint.bodyA);
                }
                if (constraint.bodyB != null) {
                    this.wakeBody(constraint.bodyB);
                }
            } else {
                this.wakeIsland(constraint.island);
            }
        }
    }

    // Implemenmtation of Conservative Advancement for two moving objects.
    void dynamicSweep(WebGLPhysicsTOIEvent toi, double timeStep, double lowerBound, double negRadius) {
        var objectA = toi.objectA;
        var objectB = toi.objectB;
        var axis = toi.axis;

        // Compute start guess on best axis.
        var vel1 = objectA.velocity;
        var axis0 = -vel1[0];
        var axis1 = -vel1[1];
        var axis2 = -vel1[2];

        var vel2 = objectB.velocity;
        axis0 += vel2[0];
        axis1 += vel2[1];
        axis2 += vel2[2];

        if (((axis0 * axis0) + (axis1 * axis1) + (axis2 * axis2)) < WebGLPhysicsConfig.DONT_NORMALIZE_THRESHOLD)
        {
            toi.toi = null;
            return;
        }

        axis[0] = axis0;
        axis[1] = axis1;
        axis[2] = axis2;

        // Compute relative linear velocity, and angular bias for distance calculations.
        var delta0 = -axis0;
        var delta1 = -axis1;
        var delta2 = -axis2;
        var angBias = 0;

        var radiusA, radiusB;
        if (!objectA.fixedRotation)
        {
            radiusA = objectA.shape.radius;
            angBias += radiusA * Math.sqrt((vel1[3] * vel1[3]) + (vel1[4] * vel1[4]) + (vel1[5] * vel1[5]));
        }

        if (!objectB.fixedRotation)
        {
            radiusB = objectB.shape.radius;
            angBias += radiusB * Math.sqrt((vel2[3] * vel2[3]) + (vel2[4] * vel2[4]) + (vel2[5] * vel2[5]));
        }

        // If relative velocity is small, then don't bother continuing as continuous detection is
        // not needed. If angular bias is too large, then we must however continue.
        if (angBias < (WebGLPhysicsConfig.CONTINUOUS_ANGULAR_BULLET / timeStep))
        {
            var radius = (radiusA < radiusB) ? radiusA : radiusB;
            radius *= WebGLPhysicsConfig.CONTINUOUS_LINEAR_BULLET / timeStep;
            if (((delta0 * delta0) + (delta1 * delta1) + (delta2 * delta2)) < (radius * radius))
            {
                toi.toi = null;
                return;
            }
        }

        var curIter = 0;
        var maxIter = 100;
        var curTOI = lowerBound;
        for (;;)
        {
            objectA.integratePosition(curTOI * timeStep);
            objectB.integratePosition(curTOI * timeStep);

            var nextContact = this.contactPairTest(toi, objectA.transform, objectB.transform);
            var seperation = nextContact;
            if (nextContact != null)
            {
                seperation += negRadius;
            }

            // objects intersecting!
            // abort!
            if (seperation == null || seperation < WebGLPhysicsConfig.GJK_EPA_DISTANCE_THRESHOLD)
            {
                if (!this.seperatingTOI(toi))
                {
                    toi.distance = nextContact;
                }
                else
                {
                    curTOI = null;
                }
                break;
            }

            // lower bound on TOI advancement.
            var dot = (axis[0] * delta0) + (axis[1] * delta1) + (axis[2] * delta2);
            var denom = (angBias - dot) * timeStep;
            if (denom <= 0)
            {
                curTOI = null;
                break;
            }

            curTOI += seperation / denom;
            if (curTOI >= 1)
            {
                curTOI = null;
                break;
            }

            curIter += 1;
            if (curIter > maxIter)
            {
                curTOI = null;
                break;
            }
        }

        toi.toi = curTOI;
    }

    // Determine if TOI event corresponds to a seperation of the
    // objects, and can be ignored.
    bool seperatingTOI(WebGLPhysicsTOIEvent toi)
    {
        var objectA = toi.objectA;
        var objectB = toi.objectB;
        var supportA = toi.closestA;
        var supportB = toi.closestB;

        var velA = objectA.velocity;
        var velB = objectB.velocity;

        var vrel0 = velA[0] - velB[0];
        var vrel1 = velA[1] - velB[1];
        var vrel2 = velA[2] - velB[2];

        if (!objectA.fixedRotation)
        {
            var relA0 = supportA[0] - objectA.transform[9];
            var relA1 = supportA[1] - objectA.transform[10];
            var relA2 = supportA[2] - objectA.transform[11];

            vrel0 += (velA[4] * relA2) - (velA[5] * relA1);
            vrel1 += (velA[5] * relA0) - (velA[3] * relA2);
            vrel2 += (velA[3] * relA1) - (velA[4] * relA0);
        }

        if (!objectB.fixedRotation)
        {
            var relB0 = supportB[0] - objectB.transform[9];
            var relB1 = supportB[1] - objectB.transform[10];
            var relB2 = supportB[2] - objectB.transform[11];

            vrel0 -= (velB[4] * relB2) - (velB[5] * relB1);
            vrel1 -= (velB[5] * relB0) - (velB[3] * relB2);
            vrel2 -= (velB[3] * relB1) - (velB[4] * relB0);
        }

        var axis = toi.axis;
        var dot = (vrel0 * axis[0]) + (vrel1 * axis[1]) + (vrel2 * axis[2]);
        return dot >= 0;
    }

    // Implemenmtation of Conservative Advancement for a moving body
    // against a static body.  (Optimised compared with dynamicSweep)
    void staticSweep(WebGLPhysicsTOIEvent toi, double timeStep, double lowerBound, double negRadius)
    {
        var objectA = toi.objectA; //dynamic
        var objectB = toi.objectB; //static
        var axis = toi.axis;

        // Compute start guess on best axis.
        var vel = objectA.velocity;
        var axis0 = -vel[0];
        var axis1 = -vel[1];
        var axis2 = -vel[2];

        if (((axis0 * axis0) + (axis1 * axis1) + (axis2 * axis2)) < WebGLPhysicsConfig.DONT_NORMALIZE_THRESHOLD)
        {
            toi.toi = null;
            return;
        }

        axis[0] = axis0;
        axis[1] = axis1;
        axis[2] = axis2;

        // Compute relative linear velocity, and angular bias for distance calculations.
        double delta0 = -axis0;
        double delta1 = -axis1;
        double delta2 = -axis2;
        double angBias = 0.0;
        if (!objectA.fixedRotationtype) {
            angBias += objectA.shape.radius * Math.sqrt((vel[3] * vel[3]) + (vel[4] * vel[4]) + (vel[5] * vel[5]));
        }

        var curIter = 0;
        var maxIter = 100;
        double curTOI = lowerBound;
        for (;;)
        {
            objectA.integratePosition(curTOI * timeStep);

            var nextContact = this.contactPairTest(toi, objectA.transform, objectB.transform);
            var seperation = nextContact;
            if (nextContact != null)
            {
                seperation += negRadius;
            }

            // objects intersecting!
            // abort!
            if (seperation == null || seperation < WebGLPhysicsConfig.GJK_EPA_DISTANCE_THRESHOLD)
            {
                if (!this.seperatingTOI(toi))
                {
                    toi.distance = nextContact;
                }
                else
                {
                    curTOI = null;
                }
                break;
            }

            // lower bound on TOI advancement.
            var dot = (axis[0] * delta0) + (axis[1] * delta1) + (axis[2] * delta2);
            var denom = (angBias - dot) * timeStep;
            if (denom <= 0)
            {
                curTOI = null;
                break;
            }

            curTOI += seperation / denom;
            if (curTOI >= 1)
            {
                curTOI = null;
                break;
            }

            curIter += 1;
            if (curIter > maxIter)
            {
                curTOI = null;
                break;
            }
        }

        toi.toi = curTOI;
    }

    int performStaticTOIBase(
        double slop,
        double timeStep,
        List<WebGLPhysicsTOIEvent> events,
        int numEvents,
        WebGLPhysicsPrivateBody objectA,
        WebGLPhysicsPrivateBody objectB) {
        var triangles = this.persistantTrianglesList;
        // Objects used in all executions of continuous collisions.
        /*if (this.continuousFakeBody == null)
        {
            this.continuousFakeBody = {
                shape : null,
                transform : VMath.m43BuildIdentity(),
                startTransform : VMath.m43BuildIdentity()
            };
            this.continuousInvTransform = VMath.m43BuildIdentity();
            this.continuousExtents = new Float32List(6);
        }*/
        var fakeBody = this.continuousFakeBody;
        var invTransform = this.continuousInvTransform;
        var extents = this.continuousExtents;

        var toi;
        //ObjectB static/kinematic.
        if (objectB.shape.type == "TRIANGLE_MESH")
        {
            var triangleArray = (objectB.shape as WebGLPhysicsTriangleMeshShape).triangleArray;
            var numTriangles, k;
            if (triangleArray.spatialMap != null)
            {
                fakeBody.shape = objectA.shape;
                // Find AABB encompassing swept shape, in local coordinate system of triangle mesh
                //VMath.m43InverseOrthonormal(objectB.transform, invTransform);
                //VMath.m43Mul(objectA.startTransform, invTransform, fakeBody.startTransform);
                //VMath.m43Mul(objectA.endTransform, invTransform, fakeBody.transform);
                objectB.transform.inverseOrthonormal(invTransform);
                objectA.startTransform.multiply(invTransform, fakeBody.startTransform);
                objectA.endTransform.multiply(invTransform, fakeBody.transform);
                calculateSweptExtents(fakeBody, extents);

                numTriangles = triangleArray.spatialMap.getOverlappingNodes(extents, triangles, 0);
                for (k = 0; k < numTriangles; k += 1) {
                    toi = WebGLPhysicsTOIEvent.allocate();
                    toi.objectA = objectA;
                    toi.objectB = objectB;
                    toi.shapeA = objectA.shape;
                    toi.shapeB = WebGLPhysicsTriangleShape.allocate();
                    (toi.shapeB as WebGLPhysicsTriangleShape).index = triangles[k].index;

                    // prevent possible GC issues
                    triangles[k] = null;

                    (toi.shapeB as WebGLPhysicsTriangleShape).triangleArray = (objectB.shape as WebGLPhysicsTriangleMeshShape).triangleArray;
                    toi.shapeB.collisionRadius = objectB.shape.collisionRadius;
                    toi.concave = true;

                    this.staticSweep(toi, timeStep, 0.0, slop);
                    if (toi.toi == null) {
                        WebGLPhysicsTOIEvent.deallocate(toi);
                        continue;
                    }

                    toi.frozenA = false;
                    toi.frozenB = true;

                    events[numEvents] = toi;
                    numEvents += 1;
                }
            } else {
                numTriangles = triangleArray.numTriangles;
                for (k = 0; k < numTriangles; k += 1)
                {
                    toi = WebGLPhysicsTOIEvent.allocate();
                    toi.objectA = objectA;
                    toi.objectB = objectB;
                    toi.shapeA = objectA.shape;
                    toi.shapeB = WebGLPhysicsTriangleShape.allocate();
                    (toi.shapeB as WebGLPhysicsTriangleShape).index = k * WebGLPhysicsPrivateTriangleArray.TRIANGLE_SIZE;
                    (toi.shapeB as WebGLPhysicsTriangleShape).triangleArray = (objectB.shape as WebGLPhysicsTriangleMeshShape).triangleArray;
                    toi.shapeB.collisionRadius = objectB.shape.collisionRadius;
                    toi.concave = true;

                    this.staticSweep(toi, timeStep, 0.0, slop);
                    if (toi.toi == null)
                    {
                        WebGLPhysicsTOIEvent.deallocate(toi);
                        continue;
                    }

                    toi.frozenA = false;
                    toi.frozenB = true;

                    events[numEvents] = toi;
                    numEvents += 1;
                }
            }
        } else {
            toi = WebGLPhysicsTOIEvent.allocate();
            toi.objectA = objectA;
            toi.objectB = objectB;
            toi.shapeA = objectA.shape;
            toi.shapeB = objectB.shape;

            this.staticSweep(toi, timeStep, 0.0, slop);
            if (toi.toi == null)
            {
                WebGLPhysicsTOIEvent.deallocate(toi);
                return numEvents;
            }

            toi.frozenA = false;
            toi.frozenB = true;

            events[numEvents] = toi;
            numEvents += 1;
        }

        return numEvents;
    }

    void update() {
        var dynamicMap = this.dynamicSpatialMap;
        var staticMap = this.staticSpatialMap;
        var rigidBodies = this.activeBodies;
        var kinematics = this.activeKinematics;
        var constraints = this.activeConstraints;
        var arbiters = this.activeArbiters;
        var gravity = this.gravity;

        var performance = this.performanceData;
        performance.discrete = 0;
        performance.sleepComputation = 0;
        performance.prestepContacts = 0;
        performance.prestepConstraints = 0;
        performance.integrateVelocities = 0;
        performance.warmstartContacts = 0;
        performance.warmstartConstraints = 0;
        performance.physicsIterations = 0;
        performance.integratePositions = 0;
        performance.continuous = 0;

        var prevTime = this.prevTimeStamp;
        if (prevTime == null)
        {
            this.prevTimeStamp = TurbulenzEngine.getTime() * 0.001;
            return;
        }

        // Compute number of sub-steps needed.
        var curTime = TurbulenzEngine.getTime() * 0.001;
        var timeDelta = (curTime - prevTime);

        var numSteps, timeStep;
        if (this.variableStep) {
            double minTimeStep = this.variableMinStep;
            double maxTimeStep = this.variableMaxStep;

            numSteps = (timeDelta / maxTimeStep).ceil();
            timeStep = (timeDelta / numSteps);

            // cap timeStep to lower bound.
            if (timeStep < minTimeStep) {
                timeStep = minTimeStep;
                numSteps = (timeDelta / timeStep).floor();
            }

            if (numSteps > this.maxSubSteps && this.maxGiveUpTimeStep != 0) {
                numSteps = (timeDelta / this.maxGiveUpTimeStep).ceil();
                timeStep = (timeDelta / numSteps);
            }
        } else {
            timeStep = this.fixedTimeStep;
            numSteps = (timeDelta / timeStep).floor();

            if (numSteps > this.maxSubSteps && this.maxGiveUpTimeStep != 0) {
                numSteps = (timeDelta / this.maxGiveUpTimeStep).ceil();
                timeStep = (timeDelta / numSteps);
            }
        }

        if (numSteps <= 0) {
            return;
        }

        // update physics time stamp regardless of
        // capping of sub step count. Otherwise time will just accumulate endlessly.
        this.prevTimeStamp += (timeStep * numSteps);

        // cap number of substeps to upper bound.
        if (numSteps > this.maxSubSteps)
        {
            numSteps = this.maxSubSteps;
        }

        this.midStep = true;

        // Determine velocities for kinematic objects.
        // And move them back to their old position (Use velocity to move it forwards in sub steps)
        var limit, i;
        var body;
        limit = kinematics.length;
        for (i = 0; i < limit;)
        {
            body = kinematics[i];
            if (!body.computeDeltaVelocity(timeStep * numSteps, body.prevTransform, body.transform, body.velocity) &&
                !body.delaySleep)
            {
                body.active = false;

                //limit -= 1;
                //kinematics[i] = kinematics[limit];
                kinematics.removeAt(i);

                this.syncBody(body);
            } else {
                body.transform.copyInto(body.newTransform);
                body.prevTransform.copyInto(body.transform);
                i += 1;
            }

            body.delaySleep = false;
        }

        // Perform substeps.
        var substep;
        for (substep = 0; substep < numSteps; substep += 1) {
            var j, extents;

            this.timeStamp += 1;
            var preTime;

            if (this.prevTimeStep == null)
            {
                this.prevTimeStep = timeStep;
            }

            var timeStepRatio = timeStep / this.prevTimeStep;
            this.prevTimeStep = timeStep;

            // ####################################################################

            // Update spatial maps with body positions and refresh inertia tensors.
            limit = rigidBodies.length;
            for (i = 0; i < limit; i += 1)
            {
                body = rigidBodies[i];

                extents = body.extents;
                body.calculateExtents(extents);
                dynamicMap.update(body, extents);

                body.refreshInertiaTensor();
            }

            limit = kinematics.length;
            for (i = 0; i < limit; i += 1)
            {
                body = kinematics[i];

                extents = body.extents;
                body.calculateExtents(extents);
                dynamicMap.update(body, extents);
            }

            // ####################################################################

            preTime = TurbulenzEngine.getTime() * 0.001;

            // Prepare broadphase
            staticMap.finalize();
            dynamicMap.finalize();

            // Perform broadphase

            // We compute first pairs of dynamic-dynamic objects
            //    objects = [ a0, a1, b0, b1, c0, c1, d0, d1 ... ]
            // We then compute pairs of dynamic-static/sleeping objects in compressed form.
            //    objects = [ ... a0, a1, a2, a3, a4, a0, ..., b0, b1, b2, b3, b4, b0 ... ]
            // where we can determine the start of a new compressed sublist by checking that
            // we are not checknig the pair (x, x)
            var objects = this.persistantObjectsList;

            // Get overlapping pairs of dynamic objects.
            var numDynDyn = dynamicMap.getOverlappingPairs(objects, 0);

            // Get overlapping pairs of static <-> dynamic objects.
            var storageIndex = numDynDyn;
            var numPairs;
            limit = rigidBodies.length;
            for (i = 0; i < limit; i += 1)
            {
                body = rigidBodies[i];
                numPairs = staticMap.getOverlappingNodes(body.extents, objects, storageIndex + 1);
                // only include sublist if number of pairs is non-zero.
                if (numPairs != 0)
                {
                    objects[storageIndex] = body;
                    storageIndex += 1 + numPairs;
                    objects[storageIndex] = body;
                    storageIndex += 1;
                }
            }

            // Get overlapping pairs of kinematic <-> sleeping dynamic
            limit = kinematics.length;
            for (i = 0; i < limit; i += 1)
            {
                body = kinematics[i];

                numPairs = staticMap.getOverlappingNodes(body.extents, objects, storageIndex + 1);
                // only include sublist if number of pairs is non-zero.
                if (numPairs != 0)
                {
                    objects[storageIndex] = body;
                    storageIndex += 1 + numPairs;
                    objects[storageIndex] = body;
                    storageIndex += 1;
                }
            }

            // Find contacts for dynamic-dynamic pairs
            // As well as kinematic-dynamic pairs.
            var objectA, objectB;
            for (i = 0; i < numDynDyn; i += 2)
            {
                objectA = objects[i];
                objectB = objects[i + 1];

                // prevent GC issues
                objects[i] = null;
                objects[i + 1] = null;
                if (!this.filtered(objectA, objectB))
                {
                    if (objectA.id < objectB.id)
                    {
                        this.narrowPhase(objectA.shape, objectB.shape, objectA, objectB);
                    }
                    else
                    {
                        this.narrowPhase(objectB.shape, objectA.shape, objectB, objectA);
                    }
                }
            }

            // Find contacts for dynamic-static/sleep pairs and kinematic/sleep-pairs
            for (i = numDynDyn; i < storageIndex;)
            {
                objectA = objects[i];
                // prevent GC issues
                objects[i] = null;

                i += 1;
                for (;;)
                {
                    objectB = objects[i];
                    //prevent GC issues
                    objects[i] = null;
                    i += 1;

                    // end of sub list.
                    if (objectA == objectB)
                    {
                        break;
                    }

                    if (!this.filtered(objectA, objectB))
                    {
                        if (objectA.id < objectB.id)
                        {
                            this.narrowPhase(objectA.shape, objectB.shape, objectA, objectB);
                        }
                        else
                        {
                            this.narrowPhase(objectB.shape, objectA.shape, objectB, objectA);
                        }
                    }
                }
            }
            performance.discrete += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################
            // Compute islands and perform sleeping.

            preTime = TurbulenzEngine.getTime() * 0.001;
            this.computeSleeping(timeStep);
            performance.sleepComputation += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################

            // Prestep arbiters
            preTime = TurbulenzEngine.getTime() * 0.001;
            i = 0;
            var arb;
            while (i < arbiters.length)
            {
                arb = arbiters[i];
                if (!arb.objectA.active && !arb.objectB.active)
                {
                    arb.active = false;
                    arbiters.removeAt(i);
                    //arbiters[i] = arbiters[arbiters.length - 1];
                    //arbiters.pop();
                    continue;
                }

                if (arb.refreshContacts())
                {
                    //arbiters[i] = arbiters[arbiters.length - 1];
                    //arbiters.pop();
                    arbiters.removeAt(i);

                    objectA = arb.objectA;
                    objectB = arb.objectB;

                    List bodyArbiters = objectA.arbiters;
                    //bodyArbiters[bodyArbiters.indexOf(arb)] = bodyArbiters[bodyArbiters.length - 1];
                    //bodyArbiters.pop();
                    bodyArbiters.remove(arb);

                    bodyArbiters = objectB.arbiters;
                    //bodyArbiters[bodyArbiters.indexOf(arb)] = bodyArbiters[bodyArbiters.length - 1];
                    //bodyArbiters.pop();
                    bodyArbiters.remove(arb);

                    if ((objectA.contactCallbacks && objectA.contactCallbacks.onRemovedContacts) ||
                        (objectB.contactCallbacks && objectB.contactCallbacks.onRemovedContacts)) {
                      this.contactCallbackRemovedArbiters.add(arb);
                    } else {
                      WebGLPhysicsArbiter.deallocate(arb);
                    }

                    continue;
                }

                arb.preStep(timeStepRatio, timeStep);

                i += 1;
            }
            performance.prestepContacts += (TurbulenzEngine.getTime() * 0.001 - preTime);

            preTime = TurbulenzEngine.getTime() * 0.001;
            // Prestep constraints
            limit = constraints.length;
            for (i = 0; i < limit; i += 1)
            {
                constraints[i].preStep(timeStepRatio, timeStep);
            }
            performance.prestepConstraints += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################

            preTime = TurbulenzEngine.getTime() * 0.001;
            // Integrate velocities, apply gravity
            limit = rigidBodies.length;
            for (i = 0; i < limit; i += 1)
            {
                body = rigidBodies[i];
                body.integrateVelocity(gravity, timeStep);
            }

            performance.integrateVelocities += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################

            preTime = TurbulenzEngine.getTime() * 0.001;
            // Warmstart arbiters
            limit = arbiters.length;
            for (i = 0; i < limit; i += 1)
            {
                arbiters[i].applyCachedImpulses();
            }
            performance.warmstartContacts += (TurbulenzEngine.getTime() * 0.001 - preTime);

            preTime = TurbulenzEngine.getTime() * 0.001;
            // Warmstart constraints
            limit = constraints.length;
            for (i = 0; i < limit; i += 1)
            {
                constraints[i].applyCachedImpulses();
            }
            performance.warmstartConstraints += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################

            preTime = TurbulenzEngine.getTime() * 0.001;
            // Physics iterations
            var numIterations = 10; //TODO: make configurable.
            for (i = 0; i < numIterations; i += 1)
            {
                limit = arbiters.length;
                for (j = 0; j < limit; j += 1)
                {
                    arbiters[j].computeAndApplyImpulses();
                }

                limit = constraints.length;
                for (j = 0; j < limit; j += 1)
                {
                    constraints[j].computeAndApplyImpulses();
                }
            }

            numIterations = 3; //TODO: make configurable.
            limit = arbiters.length;
            for (i = 0; i < numIterations; i += 1)
            {
                for (j = 0; j < limit; j += 1)
                {
                    arbiters[j].computeAndApplyBiasImpulses();
                }
            }

            performance.physicsIterations += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################

            // Apply bias velocities to get start transform for sweeps.
            // Then integrate positions to get end transform for sweeps.
            // Syncing bodies into broadphase with swept AABB.
            var unfrozen = this.persistantObjectsList2;
            var numUnfrozen = 0;

            preTime = TurbulenzEngine.getTime() * 0.001;
            limit = rigidBodies.length;
            var radius;

            var timeStepSq = timeStep * timeStep;

            var xform0, xform1;
            for (i = 0; i < limit; i += 1)
            {
                body = rigidBodies[i];
                body.applyBiasVelocities(timeStep);
                body.integratePosition(timeStep);

                // If body is moving very slowly, don't bother doing any continuous
                if (!body.isActiveVelocity(WebGLPhysicsConfig.CONTINUOUS_LINEAR_SQ / timeStep,
                                           WebGLPhysicsConfig.CONTINUOUS_ANGULAR_SQ / timeStep))
                {
                    body.sweepFrozen = true;
                    body.bullet = false;
                    continue;
                }

                // cached for triangle mesh lookups.
                //VMath.m43Copy(body.transform, body.endTransform);
                xform0 = body.transform;
                xform1 = body.endTransform;
                xform1.storage[0] = xform0.storage[0];
                xform1.storage[1] = xform0.storage[1];
                xform1.storage[2] = xform0.storage[2];
                xform1.storage[3] = xform0.storage[3];
                xform1.storage[4] = xform0.storage[4];
                xform1.storage[5] = xform0.storage[5];
                xform1.storage[6] = xform0.storage[6];
                xform1.storage[7] = xform0.storage[7];
                xform1.storage[8] = xform0.storage[8];
                xform1.storage[9] = xform0.storage[9];
                xform1.storage[10] = xform0.storage[10];
                xform1.storage[11] = xform0.storage[11];

                // determine if body should be a bullet.
                radius = body.shape.radius * WebGLPhysicsConfig.CONTINUOUS_LINEAR_BULLET;

                var vel = body.velocity;
                var vlsq = ((vel[0] * vel[0]) + (vel[1] * vel[1]) + (vel[2] * vel[2])) * timeStepSq;
                var wlsq = ((vel[3] * vel[3]) + (vel[4] * vel[4]) + (vel[5] * vel[5])) * timeStepSq;

                body.bullet = vlsq > (radius * radius) ||
                              wlsq > WebGLPhysicsConfig.CONTINUOUS_ANGULAR_BULLET;

                extents = body.extents;
                body.calculateSweptExtents(extents);
                dynamicMap.update(body, extents);

                body.sweepFrozen = false;
                unfrozen[numUnfrozen] = body;
                numUnfrozen += 1;
            }

            limit = kinematics.length;
            for (i = 0; i < limit; i += 1)
            {
                body = kinematics[i];

                body.transform.copyInto(body.startTransform);
                body.integratePosition(timeStep);

                extents = body.extents;
                body.calculateSweptExtents(extents);
                dynamicMap.update(body, extents);
            }

            performance.integratePositions += (TurbulenzEngine.getTime() * 0.001 - preTime);

            // ####################################################################

            preTime = TurbulenzEngine.getTime() * 0.001;

            // We must finalize the broadphase once more.
            // Any objects that have gone to sleep (or been woken up) will have effected
            // the static map.
            // And every dynamic object has been updated in dynamic map with its swept
            // extents for continuous collisions and absolutely must be finalized.
            staticMap.finalize();
            dynamicMap.finalize();

            // Continuous collision detection.
            var slop = WebGLPhysicsConfig.CONTINUOUS_SLOP + WebGLPhysicsConfig.CONTACT_SLOP;

            var events = this.persistantTOIEventList;
            var numEvents = 0;
            var toi;

            // Determine pairs of dynamics with one being a bullet that must be checked for collisions
            numDynDyn = dynamicMap.getOverlappingPairs(objects, 0);
            for (i = 0; i < numDynDyn; i += 2)
            {
                objectA = objects[i];
                objectB = objects[i + 1];

                // prevent possible GC issues.
                objects[i] = null;
                objects[i + 1] = null;

                if (!((objectA.bullet || objectA.kinematic) || (objectB.bullet || objectB.kinematic)) ||
                    (objectA.sweepFrozen && objectB.sweepFrozen) ||
                    this.filtered(objectA, objectB))
                {
                    continue;
                }

                if (objectA.kinematic || objectB.kinematic)
                {
                    if (objectA.kinematic)
                    {
                        numEvents = this.performStaticTOIBase(slop, timeStep, events, numEvents, objectB, objectA);
                    }
                    else
                    {
                        numEvents = this.performStaticTOIBase(slop, timeStep, events, numEvents, objectA, objectB);
                    }
                }
                else
                {
                    toi = WebGLPhysicsTOIEvent.allocate();
                    toi.objectA = objectA;
                    toi.objectB = objectB;
                    toi.shapeA = objectA.shape;
                    toi.shapeB = objectB.shape;

                    this.dynamicSweep(toi, timeStep, 0.0, slop);
                    // don't cull non-existant toi's for dynamic-dynamic.
                    // freezing of either object will impact whether a toi
                    // is able to be computed or not. miss too many collisions
                    // by culling too early here.

                    toi.frozenA = objectA.sweepFrozen;
                    toi.frozenB = objectB.sweepFrozen;

                    events[numEvents] = toi;
                    numEvents += 1;
                }
            }

            // Determine pairs of statics dynamics that must be checked for collisions
            for (i = 0; i < numUnfrozen; i += 1)
            {
                objectA = unfrozen[i];
                numPairs = staticMap.getOverlappingNodes(objectA.extents, objects, 0);
                for (j = 0; j < numPairs; j += 1)
                {
                    objectB = objects[j];
                    // prevent possible GC issues
                    objects[j] = null;

                    if (this.filtered(objectA, objectB))
                    {
                        continue;
                    }

                    numEvents = this.performStaticTOIBase(slop, timeStep, events, numEvents, objectA, objectB);
                }
            }

            // Time to begin!
            var curTimeAlpha = 0;
            while (curTimeAlpha < 1 && numEvents > 0)
            {
                var minTOI = null;
                var minIndex;

                for (i = 0; i < numEvents;)
                {
                    toi = events[i];

                    objectA = toi.objectA;
                    objectB = toi.objectB;
                    // Check if tOI Event should be culled.
                    if (objectA.sweepFrozen && objectB.sweepFrozen)
                    {
                        numEvents -= 1;
                        if (i != numEvents)
                        {
                            events[i] = events[numEvents];
                            // prevent possible GC issues
                            events[numEvents] = null;
                        }
                        WebGLPhysicsTOIEvent.deallocate(toi);
                        continue;
                    }

                    // Check if TOI Event is invalidated.
                    if ((toi.frozenA != objectA.sweepFrozen) ||
                        (toi.frozenB != objectB.sweepFrozen))
                    {
                        // Recompute TOI.
                        toi.frozenA = objectA.sweepFrozen;
                        toi.frozenB = objectB.sweepFrozen;

                        // Check if order of objects in event need swapped
                        // (For staticSweep objectA must be non-frozen)
                        if (toi.frozenA)
                        {
                            toi.objectA = objectB;
                            toi.objectB = objectA;
                            toi.shapeA = objectB.shape;
                            toi.shapeB = objectA.shape;
                            toi.frozenA = false;
                            toi.frozenB = true;
                        }
                        this.staticSweep(toi, timeStep, curTimeAlpha, slop);

                        if (toi.toi == null)
                        {
                            numEvents -= 1;
                            if (i != numEvents)
                            {
                                events[i] = events[numEvents];
                                // prevent possible GC issues
                                events[numEvents] = null;
                            }
                            WebGLPhysicsTOIEvent.deallocate(toi);
                            continue;
                        }
                    }

                    if (toi.toi != null && (minTOI == null || (toi.toi < minTOI.toi)))
                    {
                        minTOI = toi;
                        minIndex = i;
                    }

                    i += 1;
                }

                if (minTOI == null)
                {
                    break;
                }

                // remove TOI Event from list.
                numEvents -= 1;
                if (minIndex != numEvents)
                {
                    events[minIndex] = events[numEvents];
                    // prevent possible GC issues
                    events[numEvents] = null;
                }

                // Advance time alpha.
                curTimeAlpha = minTOI.toi;

                // Freeze objects at TOI.
                objectA = minTOI.objectA;
                objectB = minTOI.objectB;
                if (!objectA.collisionObject)
                {
                    if (!objectA.sweepFrozen)
                    {
                        objectA.integratePosition(timeStep * curTimeAlpha);
                        objectA.sweepFrozen = true;
                    }
                    if (objectA.permitSleep && !objectA.active)
                    {
                        this.wakeBody(objectA);
                    }
                }
                if (!objectB.collisionObject)
                {
                    if (!objectB.sweepFrozen)
                    {
                        objectB.integratePosition(timeStep * curTimeAlpha);
                        objectB.sweepFrozen = true;
                    }
                    if (objectB.permitSleep && !objectB.active)
                    {
                        this.wakeBody(objectB);
                    }
                }

                // Flip objects based on id.
                if (objectA._id > objectB._id)
                {
                    var tmp = objectA;
                    objectA = objectB;
                    objectB = tmp;

                    var tmpv = minTOI.closestA;
                    minTOI.closestA = minTOI.closestB;
                    minTOI.closestB = tmpv;

                    tmpv = minTOI.axis;
                    tmpv[0] = -tmpv[0];
                    tmpv[1] = -tmpv[1];
                    tmpv[2] = -tmpv[2];
                }

                var shapeA = objectA.shape;
                var shapeB = objectB.shape;

                // Find existing arbiter for shape pair.
                // Iterating the smaller list of either object.
                arb = null;
                var arbitersA = objectA.arbiters;
                var arbitersB = objectB.arbiters;
                var arbs = (arbitersA.length <= arbitersB.length) ? arbitersA : arbitersB;

                var numArbiters = arbs.length;
                for (i = 0; i < numArbiters; i += 1)
                {
                    var carb = arbs[i];
                    if (carb.shapeA == shapeA && carb.shapeB == shapeB &&
                        carb.objectA == objectA && carb.objectB == objectB)
                    {
                        arb = carb;
                        break;
                    }
                }

                // If arbiter does not already exist, create a new one.
                var fresh = (arb == null);
                if (fresh)
                {
                    arb = WebGLPhysicsArbiter.allocate(shapeA, shapeB, objectA, objectB);
                }

                arb.insertContact(minTOI.closestA, minTOI.closestB, minTOI.axis, minTOI.distance, minTOI.concave);
                if (fresh)
                {
                    arbiters.add(arb);
                    arb.active = true;
                    objectA.arbiters.add(arb);
                    objectB.arbiters.add(arb);
                }

                // Since object transforms do not change. The contact point which will be computed
                // in discrete collision detection at start of next world update, will be exactly
                // this contact point.
                //
                // For that reason it is a waste to perform discrete collision detection in the next update
                // for this pair of objects. This flag represent this fact.
                //
                // For active kinematic objects, the transform may change slightly due to innacuracies
                // and so not skipping discrete may achieve a new contact point and will be beneficial.
                if (!((objectA.kinematic && objectA.active) || (objectB.kinematic && objectB.active)))
                {
                    arb.skipDiscreteCollisions = true;
                }

                WebGLPhysicsTOIEvent.deallocate(minTOI);
            }

            // Prevent possible GC issues.
            while (numEvents > 0)
            {
                numEvents -= 1;
                WebGLPhysicsTOIEvent.deallocate(events[numEvents]);
                events[numEvents] = null;
            }

            // Integrate anything unfrozen to end of time step.
            while (numUnfrozen > 0)
            {
                numUnfrozen -= 1;
                objectA = unfrozen[numUnfrozen];

                // prevent possible GC issues
                unfrozen[numUnfrozen] = null;

                if (!objectA.sweepFrozen)
                {
                    objectA.integratePosition(timeStep);
                }
            }

            performance.continuous += (TurbulenzEngine.getTime() * 0.001 - preTime);
        }

        // Ensure kinematic bodies are moved 'EXACTLY' to their set transform.
        // (Numerical innacuries in integrations).
        limit = kinematics.length;
        for (i = 0; i < limit; i += 1)
        {
            body = kinematics[i];

            body.newTransform.copyInto(body.transform);
            body.newTransform.copyInto(body.prevTransform);
        }

        this.updateContactCallbacks();

        this.midStep = false;
    }

    rayTest(RayTestParameters ray) {
        var group = ray.group;
        var mask = ray.mask;
        if (group == null)
        {
            group = WebGLPhysicsDevice.FILTER_DYNAMIC;
        }
        if (mask == null)
        {
            mask = WebGLPhysicsDevice.FILTER_ALL;
        }

        var exclude = ray.exclude;

        // Create parametric ray
        var pRay = new Ray(ray.from, ray.to.clone().sub(ray.from), 1.0);
            /*{
            origin: ray.from,
            direction: VMath.v3Sub(ray.to, ray.from),
            maxFactor: 1.0
        };*/

        this.staticSpatialMap.finalize();
        this.dynamicSpatialMap.finalize();

        rayCallback(AabbTree tree, WebGLPhysicsPrivateBody obj, Ray pRay, double unusedAABBDistance, upperBound) {
            /*jshint bitwise: false*/
            var actual_obj = obj._public;
            if (actual_obj == exclude ||
                (obj.mask & group) == 0 || (obj.group & mask) == 0)
            {
                return null;
            }
            /*jshint bitwise: true*/

            pRay.maxFactor = upperBound;
            RayHit resultObj = obj.rayTest(pRay);
            if (resultObj != null)
            {
                if (obj.collisionObject)
                {
                    resultObj.collisionObject = actual_obj;
                    resultObj.body = null;
                }
                else
                {
                    resultObj.collisionObject = null;
                    resultObj.body = actual_obj;
                }
            }

            return resultObj;
        }

        var ret = AabbTree.rayTest([this.staticSpatialMap, this.dynamicSpatialMap], pRay, rayCallback);
        //delete additional factor property
        if (ret != null) {
            //delete ret.factor;
          ret.factor = null;
        }

        return ret;
    }

    //
    // cache having properties
    //   shapeA
    //   shapeB
    //   axis <-- to be mutated by this function
    //      axis is 'on' object B.
    //   closestA <-- to be populated by this function
    //   closestB <-- to be populated by this function
    contactPairTest(WebGLPhysicsNarrowCache cache, Matrix43 xformA, Matrix43 xformB)
    {
        var axis = cache.axis;
        var shapeA = cache.shapeA;
        var shapeB = cache.shapeB;
        var supportA = cache.closestA;
        var supportB = cache.closestB;

        if (this.contactGJK == null)
        {
            this.contactGJK = new WebGLGJKContactSolver();
            this.contactEPA = new WebGLContactEPA();
        }

        //
        // Special case for planes
        //
        if (shapeA.type == "PLANE" || shapeB.type == "PLANE")
        {
            WebGLPhysicsPlaneShape planeShape, otherShape;
            Matrix43 planeXForm, otherXForm;
            if (shapeA.type == "PLANE")
            {
                planeShape = shapeA;
                planeXForm = xformA;
                otherShape = shapeB;
                otherXForm = xformB;
            } else {
                planeShape = shapeB;
                planeXForm = xformB;
                otherShape = shapeA;
                otherXForm = xformA;
            }

            var A0 = planeXForm.storage[0];
            var A1 = planeXForm.storage[1];
            var A2 = planeXForm.storage[2];
            var A3 = planeXForm.storage[3];
            var A4 = planeXForm.storage[4];
            var A5 = planeXForm.storage[5];
            var A6 = planeXForm.storage[6];
            var A7 = planeXForm.storage[7];
            var A8 = planeXForm.storage[8];
            var A9 = planeXForm.storage[9];
            var A10 = planeXForm.storage[10];
            var A11 = planeXForm.storage[11];

            // local plane normal and distance.
            var n = planeShape.normal;
            var n0 = n.storage[0];
            var n1 = n.storage[1];
            var n2 = n.storage[2];
            var nd = planeShape.distance;

            // transform plane normal into world space.
            var w0 = (n0 * A0) + (n1 * A3) + (n2 * A6);
            var w1 = (n0 * A1) + (n1 * A4) + (n2 * A7);
            var w2 = (n0 * A2) + (n1 * A5) + (n2 * A8);

            A0 = otherXForm.storage[0];
            A1 = otherXForm.storage[1];
            A2 = otherXForm.storage[2];
            A3 = otherXForm.storage[3];
            A4 = otherXForm.storage[4];
            A5 = otherXForm.storage[5];
            A6 = otherXForm.storage[6];
            A7 = otherXForm.storage[7];
            A8 = otherXForm.storage[8];
            var B9 = otherXForm.storage[9];
            var B10 = otherXForm.storage[10];
            var B11 = otherXForm.storage[11];

            // transform plane into shape local space.
            n0 = (A0 * w0) + (A1 * w1) + (A2 * w2);
            n1 = (A3 * w0) + (A4 * w1) + (A5 * w2);
            n2 = (A6 * w0) + (A7 * w1) + (A8 * w2);
            nd += (w0 * (A9 - B9)) + (w1 * (A10 - B10)) + (w2 * (A11 - B11));

            // Find maximum and minimal support points on shape.
            axis[0] = n0;
            axis[1] = n1;
            axis[2] = n2;
            otherShape.localSupportWithoutMargin(axis, supportA);

            axis[0] = -n0;
            axis[1] = -n1;
            axis[2] = -n2;
            otherShape.localSupportWithoutMargin(axis, supportB);

            // Find distance from plane for each support.
            var dot1 = (supportA[0] * n0) + (supportA[1] * n1) + (supportA[2] * n2) - nd;
            var dot2 = (supportB[0] * n0) + (supportB[1] * n1) + (supportB[2] * n2) - nd;

            // Choose closest support to plane for distance computation
            // with margins.
            var seperation, c0, c1, c2;
            if ((dot1 * dot1) < (dot2 * dot2))
            {
                c0 = supportA[0];
                c1 = supportA[1];
                c2 = supportA[2];
                seperation = dot1;
            }
            else
            {
                c0 = supportB[0];
                c1 = supportB[1];
                c2 = supportB[2];
                seperation = dot2;
            }

            if ((seperation < 0) != ((dot1 * dot2) < 0))
            {
                seperation = -seperation;
                // negate normal
                w0 = -w0;
                w1 = -w1;
                w2 = -w2;
            }

            // Take collision margin from seperation.
            var rad = otherShape.collisionRadius;
            var prad = planeShape.collisionRadius;

            // find world-space support point on non-plane shape
            //VMath.m43TransformPoint(otherXForm, closest, closest);
            var a0 = (A0 * c0) + (A3 * c1) + (A6 * c2) + B9;
            var a1 = (A1 * c0) + (A4 * c1) + (A7 * c2) + B10;
            var a2 = (A2 * c0) + (A5 * c1) + (A8 * c2) + B11;

            // find world-space support point on plane shape
            // including collision margin
            var rsep = prad - seperation;
            var p0 = a0 + (w0 * rsep);
            var p1 = a1 + (w1 * rsep);
            var p2 = a2 + (w2 * rsep);

            // apply collision margin to non-plane support.
            a0 -= (w0 * rad);
            a1 -= (w1 * rad);
            a2 -= (w2 * rad);

            // apply collision radius to seperation.
            seperation -= rad + prad;

            if (shapeA.type == "PLANE") {
                axis[0] = -w0;
                axis[1] = -w1;
                axis[2] = -w2;
                supportA[0] = p0;
                supportA[1] = p1;
                supportA[2] = p2;
                supportB[0] = a0;
                supportB[1] = a1;
                supportB[2] = a2;
            } else {
                axis[0] = w0;
                axis[1] = w1;
                axis[2] = w2;
                supportA[0] = a0;
                supportA[1] = a1;
                supportA[2] = a2;
                supportB[0] = p0;
                supportB[1] = p1;
                supportB[2] = p2;
            }

            return seperation;
        } else {
            var gjk = this.contactGJK;
            var distance = gjk.evaluate(cache, xformA, xformB);
            if (distance == null)
            {
                distance = this.contactEPA.evaluate(gjk.simplex, cache, xformA, xformB);
            }

            if (distance != null)
            {
                var axis0 = axis[0];
                var axis1 = axis[1];
                var axis2 = axis[2];

                var radiusA = shapeA.collisionRadius;
                var radiusB = shapeB.collisionRadius;

                supportA[0] -= axis0 * radiusA;
                supportA[1] -= axis1 * radiusA;
                supportA[2] -= axis2 * radiusA;

                supportB[0] += axis0 * radiusB;
                supportB[1] += axis1 * radiusB;
                supportB[2] += axis2 * radiusB;

                return (distance - radiusA - radiusB);
            } else {
                return null;
            }
        }
    }


    //WebGLPhysicsNarrowCache(this.axis,this.shapeA,this.shapeB,this.closestA, this.closestB);

    // callback of the form HitResult -> Bool if callback is
    // null, then a callback of function (x) { return true; } is
    // implied.
    //
    // TODO: add type of callback?  { (rayHit: RayHit): boolean; }; ?
    RayHit convexSweepTest({
      WebGLPhysicsShape shape,
      Matrix43 from,
      Matrix43 to,
      int group: WebGLPhysicsDevice.FILTER_DYNAMIC,
      int mask: WebGLPhysicsDevice.FILTER_ALL,
      int exclude, callback}/*, [callback]*/) {
        //
        // Initialise objects reused in all convexSweepTest calls.
        //
      /*
        if (this.sweepCache == null)
        {
            this.sweepCache = {
                axis : new Vector3.zero(),
                shapeA : null,
                shapeB : null,
                closestA : new Vector3.zero(),
                closestB : new Vector3.zero()
            };

            // fake triangle shape for triangle meshes!
            this.sweepTriangle = WebGLPhysicsTriangleShape.allocate();

            this.sweepDelta = new Vector3.zero();

            this.sweepFromExtents = new Float32List(6);
            this.sweepToExtents = new Float32List(6);
            this.sweepExtents = new Float32List(6);

            // fake body used to compute shape extents in triangle mesh coordinate systems.
            this.sweepFakeBody = {
                shape : null,
                transform : null
            };
            this.sweepTransform = new Matrix43.identity();
            this.sweepTransform2 = new Matrix43.identity();
        }*/

        var cache = this.sweepCache;
        var triangle = this.sweepTriangle;
        var delta = this.sweepDelta;
        var fromExtents = this.sweepFromExtents;
        var toExtents = this.sweepToExtents;
        var extents = this.sweepExtents;
        var fakeBody = this.sweepFakeBody;
        var transform = this.sweepTransform;
        var transform2 = this.sweepTransform2;

        var that = this;
        // sweep shapeA linearlly from 'from' transform, through delta vector
        // against shapeB with transform 'transform' up to a maximum
        // distance of upperBound
        /*function */
        RayHit staticSweep(WebGLPhysicsShape shapeA, Matrix43 cpos, Vector3 delta, WebGLPhysicsShape shapeB, Matrix43 transform, double upperBound) {
            var delta0 = delta[0];
            var delta1 = delta[1];
            var delta2 = delta[2];

            var axis = cache.axis;
            var supportA = cache.closestA;
            var supportB = cache.closestB;

            //VMath.v3Neg(delta, cache.axis);
            axis[0] = -delta0;
            axis[1] = -delta1;
            axis[2] = -delta2;

            cache.shapeA = shapeA;
            cache.shapeB = shapeB;

            var distance = 0;

            var curIter = 0;
            var maxIter = 100;
            var contactDistance;

            var previousDistance = double.MAX_FINITE;//Number.MAX_VALUE;
            var intersected = false;
            for (;;)
            {
                var nextContact = that.contactPairTest(cache, cpos, transform);

                // objects intersecting!
                // abort and use previous result if existing
                if (nextContact == null || nextContact < WebGLPhysicsConfig.GJK_EPA_DISTANCE_THRESHOLD)
                {
                    if (contactDistance != null || nextContact != null)
                    {
                        if (contactDistance == null)
                        {
                            contactDistance = nextContact;
                        }
                        intersected = true;
                    }
                    break;
                }

                // terminate if distance is increasing!!
                if ((nextContact - previousDistance) >= 1)
                {
                    break;
                }
                previousDistance = nextContact;

                // distance to advance object.
                //var dot = VMath.v3Dot(delta, VMath.v3Sub(nextContact.closestB, nextContact.closestA));
                double d0 = supportB.storage[0] - supportA.storage[0];
                double d1 = supportB.storage[1] - supportA.storage[1];
                double d2 = supportB.storage[2] - supportA.storage[2];
                double dot = (delta0 * d0) + (delta1 * d1) + (delta2 * d2);

                // If seperating axis is perpendicular to direction of motion
                // Then it is not possible for use to intersect with it.
                if (dot <= WebGLPhysicsConfig.COPLANAR_THRESHOLD)
                {
                    break;
                }

                var gap = (nextContact * nextContact) / dot;
                distance += gap;
                if (distance >= upperBound)
                {
                    contactDistance = null;
                    break;
                }

                contactDistance = nextContact;
                cpos[9]  += (delta0 * gap);
                cpos[10] += (delta1 * gap);
                cpos[11] += (delta2 * gap);

                // Exit if distance between objects is nominal
                if (contactDistance <= WebGLPhysicsConfig.GJK_EPA_DISTANCE_THRESHOLD)
                {
                    intersected = true;
                    break;
                }

                // Max iteration cutoff.
                curIter += 1;
                if (curIter > maxIter)
                {
                    break;
                }
            }

            if (contactDistance == null || !intersected) {
                return null;
            } else {
                return new RayHit(supportB.clone(),axis.clone(),distance);
            }
        }

        shape = shape._private;
        //var from = from;
        //var to = to;

        //var delta = VMath.v3Sub(VMath.m43Pos(to), VMath.m43Pos(from));
        var d0 = (to[9] - from[9]);
        var d1 = (to[10] - from[10]);
        var d2 = (to[11] - from[11]);

        //var upperBound = VMath.v3Length(delta);
        var upperBound = Math.sqrt((d0 * d0) + (d1 * d1) + (d2 * d2));

        //VMath.v3Normalize(delta, delta);
        var scale = 1 / upperBound;
        delta[0] = d0 * scale;
        delta[1] = d1 * scale;
        delta[2] = d2 * scale;

        //var group = (params.group == null) ? WebGLPhysicsDevice.FILTER_DYNAMIC : params.group;
        //var mask  = (params.mask  == null) ? WebGLPhysicsDevice.FILTER_ALL     : params.mask;
        //var exclude = params.exclude;

        // Find AABB encompassing swept shape
        fakeBody.shape = shape;
        fakeBody.transform = from;
        calculateExtents.call(fakeBody, fromExtents);

        fakeBody.transform = to;
        calculateExtents.call(fakeBody, toExtents);

        //var extents = VMath.aabbUnion(fromExtents, toExtents);
        extents.min.storage[0] = (fromExtents.min.storage[0] < toExtents.min.storage[0] ? fromExtents.min.storage[0] : toExtents.min.storage[0]);
        extents.min.storage[1] = (fromExtents.min.storage[1] < toExtents.min.storage[1] ? fromExtents.min.storage[1] : toExtents.min.storage[1]);
        extents.min.storage[2] = (fromExtents.min.storage[2] < toExtents.min.storage[2] ? fromExtents.min.storage[2] : toExtents.min.storage[2]);
        extents.max.storage[0] = (fromExtents.max.storage[0] > toExtents.max.storage[0] ? fromExtents.max.storage[0] : toExtents.max.storage[0]);
        extents.max.storage[1] = (fromExtents.max.storage[1] > toExtents.max.storage[1] ? fromExtents.max.storage[1] : toExtents.max.storage[1]);
        extents.max.storage[2] = (fromExtents.max.storage[2] > toExtents.max.storage[2] ? fromExtents.max.storage[2] : toExtents.max.storage[2]);

        // Find all objects intersecting swept shape AABB.
        this.staticSpatialMap.finalize();
        this.dynamicSpatialMap.finalize();

        var objects = this.persistantObjectsList;
        var triangles = this.persistantTrianglesList;
        var staticCount = this.staticSpatialMap.getOverlappingNodes(extents, objects, 0);
        var limit = staticCount + this.dynamicSpatialMap.getOverlappingNodes(extents, objects, staticCount);

        var minResult = null;
        var i, j;
        for (i = 0; i < limit; i += 1)
        {
            var object = objects[i];
            // Prevent GC issues from persistant list.
            objects[i] = null;

            /*jshint bitwise: false*/
            // TODO: remove cast
            var actual_object = object._public;//(<any>object)._public;
            if (actual_object == exclude || object.shape == shape ||
               (object.mask & group) == 0 || (object.group & mask) == 0)
            {
                continue;
            }
            /*jshint bitwise: true*/

            RayHit result;
            var collisionShape = object.shape;
            if (collisionShape.type == "TRIANGLE_MESH")
            {
                // TODO: remove cast and fix
                var triangleArray = (collisionShape as WebGLPhysicsTriangleMeshShape).triangleArray;//(<any>collisionShape).triangleArray;
                triangle.triangleArray = triangleArray;
                // TODO: remove cast and fix
                triangle.collisionRadius = collisionShape.collisionRadius;//(<any>collisionShape).collisionRadius;

                var numTriangles;
                if (triangleArray.spatialMap)
                {
                    // Find AABB encompassing swept shape, in local coordinate system of triangle mesh.
                  object.transform.inverseOrthonormal(transform2);
                    //VMath.m43InverseOrthonormal(object.transform, transform2);

                    //VMath.m43Mul(from, transform2, transform);
                    from.multiply(transform2, transform);

                    fakeBody.transform = transform;
                    calculateExtents.call(fakeBody, fromExtents);

                    //VMath.m43Mul(to, transform2, transform);
                    to.multiply(transform2, transform);
                    calculateExtents.call(fakeBody, toExtents);

                    //var extents = VMath.aabbUnion(fromExtents, toExtents);
                    extents.min.storage[0] = (fromExtents.min.storage[0] < toExtents.min.storage[0] ? fromExtents.min.storage[0] : toExtents.min.storage[0]);
                    extents.min.storage[1] = (fromExtents.min.storage[1] < toExtents.min.storage[1] ? fromExtents.min.storage[1] : toExtents.min.storage[1]);
                    extents.min.storage[2] = (fromExtents.min.storage[2] < toExtents.min.storage[2] ? fromExtents.min.storage[2] : toExtents.min.storage[2]);
                    extents.max.storage[0] = (fromExtents.max.storage[0] > toExtents.max.storage[0] ? fromExtents.max.storage[0] : toExtents.max.storage[0]);
                    extents.max.storage[1] = (fromExtents.max.storage[1] > toExtents.max.storage[1] ? fromExtents.max.storage[1] : toExtents.max.storage[1]);
                    extents.max.storage[5] = (fromExtents.max.storage[5] > toExtents.max.storage[5] ? fromExtents.max.storage[5] : toExtents.max.storage[5]);

                    numTriangles = triangleArray.spatialMap.getOverlappingNodes(extents, triangles, 0);
                    for (j = 0; j < numTriangles; j += 1)
                    {
                        triangle.index = triangles[j].index;
                        // avoid GC problems of persistant array.
                        triangles[j] = null;

                        //VMath.m43Copy(from, transform2);
                        from.copyInto(transform2);
                        result = staticSweep(shape, transform2, delta, triangle, object.transform, upperBound);
                        if (result != null)
                        {
                            result.collisionObject = actual_object;
                            result.body = null;

                            if (!callback || callback(result))
                            {
                                minResult = result;
                                upperBound = result.distance;
                            }
                        }
                    }
                } else {
                    numTriangles = triangleArray.numTriangles;
                    for (j = 0; j < numTriangles; j += 1) {
                        triangle.index = (j * WebGLPhysicsPrivateTriangleArray.TRIANGLE_SIZE);
                        //VMath.m43Copy(from, transform2);
                        from.copyInto(transform2);
                        result = staticSweep(shape, transform2, delta, triangle, object.transform, upperBound);
                        if (result != null ) {
                            result.collisionObject = actual_object;
                            result.body = null;

                            if (!callback || callback(result))
                            {
                                minResult = result;
                                upperBound = result.distance;
                            }
                        }
                    }
                }
            } else {
                from.copyInto(transform2);
                result = staticSweep(shape, transform2, delta, collisionShape, object.transform, upperBound);
                if (result != null) {
                    // TODO: remove cast and fix
                    if (object.collisionObject) { //((<any>object).collisionObject)
                        result.collisionObject = actual_object;
                        result.body = null;
                    } else {
                        result.collisionObject = null;
                        result.body = actual_object;
                    }

                    if (!callback || callback(result))
                    {
                        minResult = result;
                        upperBound = result.distance;
                    }
                }
            }

            // Cut off on epsilon distance
            // Based on rough experimental result
            if (upperBound < 1e-4) {
                // clean up remaining objects for GC
                for (j = i; j < limit; j += 1)
                {
                    objects[j] = null;
                }

                break;
            }
        }

        if (minResult) {
            // delete additional property
            //delete minResult.distance;
          minResult.distance = null;
        }

        return minResult;
    }

    bool addBody(WebGLPhysicsPrivateBody body) {
        if (body.world == null) {
            return false;
        }

        body.world = this;
        if (body.collisionObject && !body.kinematic) {
            this.collisionObjects.add(body);
            this.syncBody(body);
            return true;
        }
        if (body.kinematic) {
            this.kinematicBodies.add(body);
        } else {
            this.rigidBodies.add(body);
        }

        var addSleeping = !body.active;
        body.previouslyActive = true;
        body.active = false;

        // Prepare body for disjoint set forest algorithm
        // in computeSleeping
        body.islandRoot = body;
        body.islandRank = 0;

        if (!addSleeping)
        {
            this.wakeBody(body);
        } else {
            this.syncBody(body);
        }

        return true;
    }

    bool removeBody(WebGLPhysicsPrivateBody body) {
        if (body.world != this) {
            return false;
        }

        List list, activeList;
        if (body.collisionObject && !body.kinematic)
        {
            list = this.collisionObjects;
        }
        else if (body.kinematic)
        {
            list = this.kinematicBodies;
            activeList = this.activeKinematics;
        }
        else
        {
            list = this.rigidBodies;
            activeList = this.activeBodies;
        }

        body.world = null;
        //list[list.indexOf(body)] = list[list.length - 1];
        //list.pop();
        list.remove(body);

        if (activeList && body.active)
        {
            //activeList[activeList.indexOf(body)] = activeList[activeList.length - 1];
            //activeList.pop();
            activeList.remove(body);
            this.dynamicSpatialMap.remove(body);
        }
        else
        {
            this.staticSpatialMap.remove(body);
        }

        this.removeArbitersFromObject(body);

        this.removeFromContactCallbacks(body);

        var island = body.island;
        if (island) {
            List bodies = island.bodies;
            var bodyIndex = bodies.indexOf(body);
            if (bodyIndex != -1) {
                //bodies[bodyIndex] = bodies[bodies.length - 1];
                //bodies.pop();
              bodies.removeAt(bodyIndex);
            }
            body.island = null;
        }

        return true;
    }

    bool addConstraint(WebGLPhysicsPrivateBody constraint) {
        if (constraint.world) {
            return false;
        }

        constraint.world = this;
        this.constraints.add(constraint);

        if (constraint.bodyA != null)
        {
            constraint.bodyA.constraints.add(constraint);
        }
        if (constraint.bodyB != null)
        {
            constraint.bodyB.constraints.add(constraint);
        }

        var addSleeping = !constraint.active;
        constraint.active = false;

        // Prepare constraint for disjoint set forest algorithm
        // in computeSleeping
        constraint.islandRoot = constraint;
        constraint.islandRank = 0;

        if (!addSleeping)
        {
            this.wakeConstraint(constraint);
        }

        return true;
    }

    bool removeConstraint(WebGLPhysicsConstraint constraint) {
      if (constraint.world != this) {
          return false;
      }

      constraint.world = null;

      List list = this.constraints;
      //list[list.indexOf(constraint)] = list[list.length - 1];
      //list.pop();
      list.remove(constraint);

      if (constraint.bodyA != null)
      {
          list = constraint.bodyA.constraints;
          //list[list.indexOf(constraint)] = list[list.length - 1];
          //list.pop();
          list.remove(constraint);
      }
      if (constraint.bodyB != null)
      {
          list = constraint.bodyA.constraints;
          //list[list.indexOf(constraint)] = list[list.length - 1];
          //list.pop();
          list.remove(constraint);
      }

      if (constraint.active)
      {
          list = this.activeConstraints;
          //list[list.indexOf(constraint)] = list[list.length - 1];
          //list.pop();
          list.remove(constraint);
      }

      var island = constraint.island;
      if (island != null)
      {
          List constraints = island.constraints;
          int constraintIndex = constraints.indexOf(constraint);
          if (constraintIndex != -1) {
              //constraints[constraintIndex] = constraints[constraints.length - 1];
              //constraints.pop();
            constraints.removeAt(constraintIndex);
          }
          constraint.island = null;
      }

      return true;
    }

    void flush() {
      // Use public remove# methods to ensure necessary side effects
      // Occur and avoid code duplication.

      while (this.rigidBodies.length > 0)
      {
          this.removeBody(this.rigidBodies[0]);
      }

      while (this.collisionObjects.length > 0)
      {
          this.removeBody(this.collisionObjects[0]);
      }

      while (this.kinematicBodies.length > 0)
      {
          this.removeBody(this.kinematicBodies[0]);
      }

      while (this.constraints.length > 0)
      {
          this.removeConstraint(this.constraints[0]);
      }

      this.timeStamp = 0.0;
    }

    void removeArbitersFromObject(WebGLPhysicsPrivateBody object) {
        var arbiters = object.arbiters;
        var worldArbiters = this.activeArbiters;
        while (arbiters.length > 0)
        {
            var arb = arbiters.removeLast();

            // Remove from other object also.
            List bodyArbiters = (arb.objectA == object) ? arb.objectB.arbiters : arb.objectA.arbiters;
            //bodyArbiters[bodyArbiters.indexOf(arb)] = bodyArbiters[bodyArbiters.length - 1];
            // bodyArbiters.pop()
            bodyArbiters.remove(arb);

            // Remove from world arbiters list
            if (arb.active)
            {
                //worldArbiters[worldArbiters.indexOf(arb)] = worldArbiters[worldArbiters.length - 1];
                //worldArbiters.pop();
              worldArbiters.remove(arb);
            }

            // Clean up all contacts.
            while (arb.contacts.length > 0)
            {
                var contact = arb.contacts.removeLast();
                WebGLPhysicsContact.deallocate(contact);
            }

            WebGLPhysicsArbiter.deallocate(arb);
        }
    }

    void removeFromContactCallbacks(WebGLPhysicsPrivateBody object) {
        var contactCallbackObjects = this.contactCallbackObjects;
        var numObjects = contactCallbackObjects.length;
        var n;
        for (n = 0; n < numObjects; n += 1)
        {
            if (contactCallbackObjects[n] == object)
            {
                numObjects -= 1;
                if (n < numObjects)
                {
                    contactCallbackObjects[n] = contactCallbackObjects[numObjects];
                }
                contactCallbackObjects.length = numObjects;
                break;
            }
        }
        object.addedToContactCallbacks = false;
    }

    void updateContactCallbacks() {
        var contactCallbackObjects = this.contactCallbackObjects;
        var numObjects = contactCallbackObjects.length;
        var publicContacts = WebGLPhysicsContact.publicContacts;
        var callbackContacts = WebGLPhysicsContact.callbackContacts;
        var arbiter, objectA, objectB, contactCallbacksA, contactCallbacksB;
        var n = 0;
        while (n < numObjects)
        {
            var object = contactCallbackObjects[n];
            var arbiters = object.arbiters;
            var numArbiters = arbiters.length;
            if (0 == numArbiters)
            {
                object.contactCallbacks.added = false;
                numObjects -= 1;
                if (n < numObjects)
                {
                    contactCallbackObjects[n] = contactCallbackObjects[numObjects];
                }
                contactCallbackObjects.length = numObjects;
            }
            else
            {
                var i, j;
                for (i = 0; i < numArbiters; i += 1)
                {
                    arbiter = arbiters[i];
                    if (0 != arbiter.contactFlags)
                    {
                        var contacts = arbiter.contacts;
                        var numContacts = contacts.length;

                        while (publicContacts.length < numContacts)
                        {
                            publicContacts[publicContacts.length] = new WebGLPhysicsPublicContact();
                        }

                        callbackContacts.length = numContacts;
                        for (j = 0; j < numContacts; j += 1)
                        {
                            var publicContact = publicContacts[j];
                            publicContact._private = contacts[j];
                            callbackContacts[j] = publicContact;
                        }

                        objectA = arbiter.objectA;
                        objectB = arbiter.objectB;

                        contactCallbacksA = objectA.contactCallbacks;
                        contactCallbacksB = objectB.contactCallbacks;

                        /*jshint bitwise: false*/
                        if (arbiter.contactFlags & 1)
                        {
                            if (null != contactCallbacksA && contactCallbacksA.onAddedContacts)
                            {
                                contactCallbacksA.onAddedContacts(objectA._public, objectB._public, callbackContacts);
                            }
                            if (null != contactCallbacksB && contactCallbacksB.onAddedContacts)
                            {
                                contactCallbacksB.onAddedContacts(objectA._public, objectB._public, callbackContacts);
                            }
                        }

                        if (arbiter.contactFlags & 2)
                        {
                            if (null != contactCallbacksA && contactCallbacksA.onProcessedContacts)
                            {
                                contactCallbacksA.onProcessedContacts(objectA._public, objectB._public, callbackContacts);
                            }
                            if (null != contactCallbacksB && contactCallbacksB.onProcessedContacts)
                            {
                                contactCallbacksB.onProcessedContacts(objectA._public, objectB._public, callbackContacts);
                            }
                        }

                        if (arbiter.contactFlags & 4)
                        {
                            if (null != contactCallbacksA && contactCallbacksA.onRemovedContacts)
                            {
                                contactCallbacksA.onRemovedContacts(objectA._public, objectB._public, callbackContacts);
                            }
                            if (null != contactCallbacksB && contactCallbacksB.onRemovedContacts)
                            {
                                contactCallbacksB.onRemovedContacts(objectA._public, objectB._public, callbackContacts);
                            }
                        }

                        arbiter.contactFlags = 0;

                        // Flag contacts as old
                        for (j = 0; j < numContacts; j += 1)
                        {
                            contacts[j][51] = 0;
                        }
                    }
                }
                n += 1;
            }
        }

        // Callbacks for pairs no longer touching
        var contactCallbackRemovedArbiters = this.contactCallbackRemovedArbiters;
        numObjects = contactCallbackRemovedArbiters.length;
        callbackContacts.length = 0;
        for (n = 0; n < numObjects; n += 1) {
            arbiter = contactCallbackRemovedArbiters[n];

            objectA = arbiter.objectA;
            objectB = arbiter.objectB;

            contactCallbacksA = objectA.contactCallbacks;
            contactCallbacksB = objectB.contactCallbacks;

            if (null != contactCallbacksA && contactCallbacksA.onRemovedContacts) {
                contactCallbacksA.onRemovedContacts(objectA._public, objectB._public, callbackContacts);
            }
            if (null != contactCallbacksB && contactCallbacksB.onRemovedContacts) {
                contactCallbacksB.onRemovedContacts(objectA._public, objectB._public, callbackContacts);
            }

            WebGLPhysicsArbiter.deallocate(arbiter);
        }
        contactCallbackRemovedArbiters.length = 0;
    }
}