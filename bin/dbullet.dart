library debullet_test;
import 'dart:async';
import 'package:dullet_physics/dullet_physics.dart';
import 'package:vector_math/vector_math.dart';

WebGLPhysicsWorld world;
WebGLPhysicsCollisionObject obj;
WebGLPhysicsRigidBody rigid;

DateTime _startTime;
List<WebGLPhysicsRigidBody> boxBodies = <WebGLPhysicsRigidBody>[];

double currentTime() {
  return new DateTime.now().difference(_startTime).inMicroseconds.toDouble();
}

void main() {
  _startTime = new DateTime.now();
  world = new WebGLPhysicsWorld(currentTime);
  //world.update();


  // Specify the generic settings for the collision objects
  var collisionMargin = 0.005;
  var mass = 20.0;
  var numCubes = 6;

  var cubeExtents = new Vector3(0.5, 0.5, 0.5);

  var floorShape = new WebGLPhysicsPlaneShape(
    normal : new Vector3(0.0, 1.0, 0.0),
    distance : 0.0,
    margin : collisionMargin
  );

  var floorObject = new WebGLPhysicsCollisionObject(
    shape : floorShape,
    transform : new Matrix43.identity(),
    friction : 0.5,
    restitution : 0.3,
    group: WebGLPhysicsDevice.FILTER_STATIC,
    mask: WebGLPhysicsDevice.FILTER_ALL
  );

  // Adds the floor collision object to the physicsDevice
  world.addCollisionObject(floorObject);

  var boxShape = new WebGLPhysicsBoxShape(cubeExtents,
    //halfExtents : cubeExtents,
    margin : collisionMargin
  );


  var inertia = boxShape.inertia.scale(mass);
  //= mathDevice.v3ScalarMul(inertia, mass);



  // Initial box is created as a rigid body
  var prb = new WebGLPhysicsRigidBody(
    shape : boxShape,
    mass : mass,
    inertia : inertia,
    transform : new Matrix43.fromTranslation(0.0, 1.0, 0.0),
    friction : 0.5,
    restitution : 0.3,
    angularDamping: 0.9,
    frozen : false
  );
  boxBodies.add(prb);
  world.addRigidBody(prb);


  for (var n = 1; n < numCubes; n += 1) {
    // Each additional box is cloned from the original
    var newBox = prb.clone();
    boxBodies.add(newBox);
    newBox.transform = new Matrix43.fromTranslation(0.0, 1.0 + 1.5 * n, 0.0);
    world.addRigidBody(newBox);
  }

  while(true) {
    print('update...');
    world.update();
    //print(currentTime());
    print(boxBodies.last.transform);
  }


  //Timer.run(update);
  print('done');
}




void update() {


  Timer.run(update);
  print('update');
}
