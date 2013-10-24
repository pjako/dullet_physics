part of dullet_physics;
//
// WebGL Physics Device
//
class WebGLPhysicsDevice /*implements PhysicsDevice*/ {
    static const int FILTER_DYNAMIC = 1; // 1 (on prototype)
    static const int FILTER_STATIC = 2; // 2 (on prototype)
    static const int FILTER_KINEMATIC = 4; // 4 (on prototype)
    static const int FILTER_DEBRIS = 8; // 8 (on prototype)
    static const int FILTER_TRIGGER = 16; // 16 (on prototype)
    static const int FILTER_CHARACTER = 32; // 32 (on prototype)
    static const int FILTER_PROJECTILE = 64; // 64 (on prototype)
    static const int FILTER_USER_MIN = 128; // 128 (on prototype)
    static const int FILTER_USER_MAX = 0x8000; // 0x8000 (on prototype)
    static const int FILTER_ALL = 0xffff; // 0xffff (on prototype)

    static const int version     = 1;
    static const String vendor   = "Turbulenz";

    WebGLPhysicsDevice() {
        //this.genObjectId = 0;
    }

    /*static create(/* params */) : WebGLPhysicsDevice
    {
        return new WebGLPhysicsDevice();
    }*/

    WebGLPhysicsWorld createDynamicsWorld(params) {
        return new WebGLPhysicsWorld(params);
    }

    WebGLPhysicsPlaneShape createPlaneShape(params) {
      return new WebGLPhysicsPlaneShape(params);
    }

    WebGLPhysicsBoxShape createBoxShape(params) {
      return new WebGLPhysicsBoxShape(params);
    }

    WebGLPhysicsSphereShape createSphereShape(params) {
      return new WebGLPhysicsSphereShape(params);
    }

    WebGLPhysicsCapsuleShape createCapsuleShape(params) {
      return new WebGLPhysicsCapsuleShape(params);
    }

    WebGLPhysicsCylinderShape createCylinderShape(params) {
      return new WebGLPhysicsCylinderShape(params);
    }

    WebGLPhysicsConeShape createConeShape(params){
      return new WebGLPhysicsConeShape(params);
    }

    WebGLPhysicsTriangleMeshShape createTriangleMeshShape(params) {
     return new WebGLPhysicsTriangleMeshShape(params);
    }

    WebGLPhysicsConvexHullShape createConvexHullShape(params) {
      return new WebGLPhysicsConvexHullShape(params);
    }

    WebGLPhysicsTriangleArray createTriangleArray(params) {
      return new WebGLPhysicsTriangleArray(params);
    }

    WebGLPhysicsCollisionObject createCollisionObject(params) {
      return new WebGLPhysicsCollisionObject(params);
    }

    WebGLPhysicsRigidBody createRigidBody(params) {
      return new WebGLPhysicsRigidBody(params);
    }

    createPoint2PointConstraint(params) {
      return new WebGLPhysicsPoint2PointConstraint(params);
    }

    PhysicsHingeConstraint createHingeConstraint(params) {
      return WebGLPhysicsConstraint.create("HINGE", params);
    }

    PhysicsConeTwistConstraint createConeTwistConstraint(params) {
      return WebGLPhysicsConstraint.create("CONETWIST", params);
    }

    Physics6DOFConstraint create6DOFConstraint(params) {
      return WebGLPhysicsConstraint.create("D6", params);
    }

    PhysicsSliderConstraint createSliderConstraint(params) {
      return WebGLPhysicsConstraint.create("SLIDER", params);
    }

    WebGLPhysicsCharacter createCharacter(params) {
      return new WebGLPhysicsCharacter(params);
    }
}