part of dullet_physics;

abstract class WebGLPhysicsShape {
  // From Physics Shape Father class
  //final String type;
  double _margin;
  double _radius;
  double _collisionRadius;
  Vector3 _halfExtents;
  Vector3 _inertia;
  Vector3 get inertia => _inertia.clone();
  // Ours
  Vector3 _center;
  RayHit rayTest(var ray);
  //var _private;

  void localSupportWithoutMargin(Vector3 vec, Vector3 dst) {

  }

  double get margin => _collisionRadius;

  void set margin(double margin_) {
    _halfExtents[0] += (margin_ - _collisionRadius);
    _halfExtents[1] += (margin_ - _collisionRadius);
    _halfExtents[2] += (margin_ - _collisionRadius);
    _radius += (margin_ - _collisionRadius);
    _collisionRadius = margin;
  }

  Vector3 getInertia(Vector3 out) {
    _inertia.copyInto(out);
  }
  Vector3 get intertia => _inertia.clone();

  double get radius => _radius;


/*
  double get margin => _private.collisionRadius;
  void set margin(double margin) {
    var pr = _private;
    pr.halfExtents[0] += (margin - pr.collisionRadius);
    pr.halfExtents[1] += (margin - pr.collisionRadius);
    pr.halfExtents[2] += (margin - pr.collisionRadius);
    pr.radius += (margin - pr.collisionRadius);

    pr.collisionRadius = margin;
  }
  Vector3 get halfExtents => _private.halfExtents.clone();
  Vector3 get inertia => _private.inertia.clone();
  double get radius => _private.radius;
*/


// From orgin
/*
   *     static version = 1;

    // From Physics Shape

    margin      : number;
    radius      : number;
    halfExtents : any; // v3
    inertia     : any; // v3
    type        : string;

    // Ours

    rayTest     : (ray: any) => WebGLPhysicsShapeRayTestResult;
    center      : any; // v3
    _private    : any;

    // TODO: inherit the subclasses rather than using _public /
    // _private membes
   */

}
