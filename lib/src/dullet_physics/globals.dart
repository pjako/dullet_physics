part of dullet_physics;

class WebGLPhysicsConfig {
  // (Contact physics)
  // Amount of slop permitted in contact penetration
  // And percentage of positional error to resolve
  // per simulation step.
  static const double CONTACT_SLOP = 0.015;
  static const double CONTACT_BAUMGRAUTE = 0.35;
  static const double CONTACT_STATIC_BAUMGRAUTE = 0.65; // different coeffecient for static-dynamic case.
  //
  // (Contact persistance)
  // Amount of seperation permitted before contact cache is destroyed
  static const double CONTACT_MAX_Y_SEPERATION = 0.05;
  // Amount of squared tangential seperation permitted before contact cache is destroyed
  static const double CONTACT_MAX_SQ_XZ_SEPERATION = 2 * (0.245 * 0.245);
  // Amount of seperation permitted for a new contact to inherit existing cache's impulse cache
  static const double CONTACT_INHERIT_SQ_SEPERATION = 3 * (0.75 * 0.75); //always inherit closest
  // Amount of seperation to assume another contacts place instead of just inheriting
  static const double CONTACT_EQUAL_SQ_SEPERATION = 3 * (0.001 * 0.001);
  //
  // (Collision detection)
  // seperation distance to assume objects are infact intersecting.
  static const double GJK_EPA_DISTANCE_THRESHOLD = 1e-4;
  // fractional change in computed distance at which we may terminate GJK algorithm.
  static const double GJK_FRACTIONAL_THRESHOLD = 1e-4;
  //
  // Threshold for the square of the ratio of velocity to radius of an object to be considered
  // moving fast enough to be collided continuously against static/sleeping objects.
  // this is multiplied with time step.
  static const double CONTINUOUS_LINEAR_SQ = 0.35;
  // Threshold for square of angular velocity to be considered moving fast enough to be collided
  // continuously against static/sleeping objects.
  // this is multiplied with time step.
  static const double CONTINUOUS_ANGULAR_SQ = 0.25;
  // Threshold for ratio of squared linear speed to radius for object to be considered moving fast enough
  // to be collided continuously against other dynamic objects.
  // This is a 'per-step' ratio.
  static const double CONTINUOUS_LINEAR_BULLET = 0.75;
  // Threshold for squared angular speed to be considered for continuous collisions against other dynamics.
  // This is a 'per-step' value.
  static const double CONTINUOUS_ANGULAR_BULLET = 0.5;
  // Amount of extra slop permitted in continuous collisions.
  // This is added ontop of the usual contact slop.
  static const double CONTINUOUS_SLOP = 0.015;
  //
  // (Sleeping)
  // Threshold for the square of the ratio of velocity to radius of an object to
  // be considered at rest. Eg: if threshold is 1; then in a given second should the object
  // move less than 1x its radius; it will be considered at rest.
  static const double SLEEP_LINEAR_SQ = 0.01;
  // squared angular velocity to be considered 'at rest'.
  // There is no scaling; as we base this on tangentenial velocity of body at radius which means
  // that when computing the ratio w.r.t to radius we end up simply with angular velocity.
  static const double SLEEP_ANGULAR_SQ = 0.1;
  // number of world updates body must be 'at rest' to be allowed to sleep.
  static int SLEEP_DELAY = 60;
  //
  // (Misc)
  static const double MAX_ANGULAR = Math.PI; //maximum angular velocity per time-step before clamping in integration occurs.
  //
  // (General)
  static const double QUADRATIC_THRESHOLD = 1e-8;
  static const double DONT_NORMALIZE_THRESHOLD = 1e-8;
  static const double COLLINEAR_THRESHOLD = 1e-10;
  static const double COPLANAR_THRESHOLD = 1e-16;
}