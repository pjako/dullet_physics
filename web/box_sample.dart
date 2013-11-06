import 'dart:html';
import 'dart:typed_data';
import 'package:game_loop/game_loop_html.dart';
import'package:spectre/spectre.dart';

import 'package:dullet_physics/dullet_physics.dart';
import 'package:vector_math/vector_math.dart';

const double collisionMargin = 0.005;
const double mass = 20.0;
const int numCubes = 50;

WebGLPhysicsWorld world;
WebGLPhysicsCollisionObject obj;
WebGLPhysicsRigidBody rigid;
List<WebGLPhysicsRigidBody> boxBodies = <WebGLPhysicsRigidBody>[];



final String vertexShader =
'''
precision highp float;

attribute vec3 POSITION;
attribute vec2 TEXCOORD0;

uniform mat4 cameraTransform;

varying vec2 samplePoint;


void main() {
    samplePoint = TEXCOORD0;
    vec4 vPosition4 = vec4(POSITION.x, POSITION.y, POSITION.z, 1.0);
    gl_Position = cameraTransform * vPosition4;
}
''';
final String fragmentShader =
'''
precision mediump float;

varying vec2 samplePoint;

uniform sampler2D diffuse;

void main() {
  vec3 diffuseColor = vec3(texture2D(diffuse, samplePoint));
  gl_FragColor = vec4(diffuseColor, 1.0);
}
''';
final Float32List boxVertArray = new Float32List.fromList([
  -0.5, -0.5,  0.5, 0.0, 0.0,
   0.5, -0.5,  0.5, 1.0, 0.0,
   0.5,  0.5,  0.5, 1.0, 1.0,
  -0.5,  0.5,  0.5, 0.0, 1.0,
  -0.5,  0.5,  0.5, 0.0, 0.0,
   0.5,  0.5,  0.5, 1.0, 0.0,
   0.5,  0.5, -0.5, 1.0, 1.0,
  -0.5,  0.5, -0.5, 0.0, 1.0,
  -0.5,  0.5, -0.5, 1.0, 1.0,
   0.5,  0.5, -0.5, 0.0, 1.0,
   0.5, -0.5, -0.5, 0.0, 0.0,
  -0.5, -0.5, -0.5, 1.0, 0.0,
  -0.5, -0.5, -0.5, 0.0, 0.0,
   0.5, -0.5, -0.5, 1.0, 0.0,
   0.5, -0.5,  0.5, 1.0, 1.0,
  -0.5, -0.5,  0.5, 0.0, 1.0,
   0.5, -0.5,  0.5, 0.0, 0.0,
   0.5, -0.5, -0.5, 1.0, 0.0,
   0.5,  0.5, -0.5, 1.0, 1.0,
   0.5,  0.5,  0.5, 0.0, 1.0,
  -0.5, -0.5, -0.5, 0.0, 0.0,
  -0.5, -0.5,  0.5, 1.0, 0.0,
  -0.5,  0.5,  0.5, 1.0, 1.0,
  -0.5,  0.5, -0.5, 0.0, 1.0 ]);
final Uint16List boxIdxArray = new Uint16List.fromList([
  2,  0,  1,
  3,  0,  2,
  6,  4,  5,
  7,  4,  6,
  10,  8,  9,
  11,  8, 10,
  14, 12, 13,
  15, 12, 14,
  18, 16, 17,
  19, 16, 18,
  22, 20, 21,
  23, 20, 22
  ]);


final CanvasElement frontBuffer = query('#front_buffer');
GameLoopHtml gameLoop;

GraphicsDevice device;
GraphicsContext context;
DebugDrawManager debug;

Camera camera;


RasterizerState rasterizationState;
DepthState depthState;

ShaderProgram boxShader;
SingleArrayIndexedMesh boxMesh;
InputLayout boxInputLayout;
Texture2D boxTexture;
SamplerState boxSampler;
Viewport viewPort;







void main() {
  gameLoop = new GameLoopHtml(frontBuffer);
  gameLoop.onUpdate = update;
  gameLoop.onRender = render;
  initGraphics();
  initDbullet();
}

void initGraphics() {
  device = new GraphicsDevice(frontBuffer);
  context = device.context;
  viewPort = new Viewport.bounds(0,0,frontBuffer.width,frontBuffer.height);

  debug = new DebugDrawManager(device);
  boxTexture = new Texture2D('box-texture',device);


  rasterizationState = new RasterizerState.cullClockwise();
  depthState = new DepthState.depthWrite();



  boxSampler = new SamplerState.linearWrap('box-sampler', device);

  boxTexture.uploadFromURL('assets/textures/crate.png').then((onValue){start();});
  boxMesh =  new SingleArrayIndexedMesh('box-mesh',device);

  var vertArr = boxVertArray;
  boxMesh.vertexArray.uploadData(vertArr, UsagePattern.StaticDraw);
  var idxArr = boxIdxArray;
  boxMesh.indexArray.uploadData(idxArr, UsagePattern.StaticDraw);
  boxMesh.count = idxArr.length;

  boxMesh.attributes['POSITION'] = new SpectreMeshAttribute( 'POSITION',
      new VertexAttribute(0, 0, 0, 5*4, DataType.Float32, 3, false));
  boxMesh.attributes['TEXCOORD0'] = new SpectreMeshAttribute( 'TEXCOORD0',
      new VertexAttribute(0, 0, 3*4, 5*4, DataType.Float32, 2, false));

  boxShader = new ShaderProgram('box-program', device);
  var vert = new VertexShader('box-vertex', device)
  ..source = vertexShader;
  var frag = new FragmentShader('box-fragment', device)
  ..source = fragmentShader;



  boxShader.vertexShader = vert;
  boxShader.fragmentShader = frag;
  boxShader.link();


  boxInputLayout = new InputLayout('box-inputLayout', device);
  boxInputLayout.mesh = boxMesh;
  boxInputLayout.shaderProgram = boxShader;
  print(boxMesh.attributes);


  camera = new Camera();
  camera.position = new Vector3(7.0,5.0,7.0);
  camera.focusPosition = new Vector3.zero();
  camera.aspectRatio = frontBuffer.width.toDouble()/frontBuffer.height.toDouble();



}

void initDbullet() {
  world = new WebGLPhysicsWorld(() => gameLoop.gameTime * 1000.0);


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

  world.addCollisionObject(floorObject);

  var boxShape = new WebGLPhysicsBoxShape(
      cubeExtents,
      margin : collisionMargin
  );

  var inertia = boxShape.inertia.scale(mass);

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
    newBox.userData = n;
    boxBodies.add(newBox);
    newBox.transform = new Matrix43.fromTranslation(0.0, 1.0 + 1.5 * n, 0.0);
    world.addRigidBody(newBox);
  }

}

void start() {
  gameLoop.start();

}

void update(GameLoopHtml loop) {
  debug.update(loop.dt);
  world.update();
  //print(gameLoop.gameTime);
}

final Matrix4 projectionView = new Matrix4.zero();
final Matrix4 view = new Matrix4.zero();
final Matrix43 boxTransform = new Matrix43();
Matrix4 boxTransform4 = new Matrix4.zero();
void render(GameLoopHtml loop) {
  context.clearColorBuffer(0.66, 0.66, 0.66, 1.0);
  context.clearDepthBuffer(1.0);
  context.reset();

  context.setViewport(viewPort);
  camera.copyProjectionMatrix(projectionView);
  camera.copyViewMatrix(view);
  projectionView.multiply(view);
  //debug.addCross(new Vector3.zero(), DebugDrawManager.ColorGreen);


  context.setRasterizerState(rasterizationState);
  context.setDepthState(depthState);

  //box.calculateTransform(boxTransform);

  for(var box in boxBodies) {

    box.calculateTransform(boxTransform);
    boxTransform4 = boxTransform.multiplyMatrix4(projectionView, boxTransform4);

    context.setPrimitiveTopology(PrimitiveTopology.Triangles);

    context.setShaderProgram(boxShader);

    context.setInputLayout(boxInputLayout);
    context.setIndexedMesh(boxMesh);

    context.setConstant('cameraTransform', boxTransform4.storage);

    context.setSampler(0, boxSampler);
    context.setTexture(0, boxTexture);
    context.drawIndexedMesh(boxMesh);
    debug.addCross(box.position, DebugDrawManager.ColorGreen);

  }
  debug.prepareForRender();
  debug.render(camera);

}

/*
    /*jshint white: false*/
    // Vertex buffer parameters for crate
    var vertexbufferParameters =
    {
        numVertices: 24,
        attributes: ['FLOAT3', 'SHORT2'],
        dynamic: false,
        data: [
                -0.5, -0.5,  0.5, 0, 0,
                 0.5, -0.5,  0.5, 1, 0,
                 0.5,  0.5,  0.5, 1, 1,
                -0.5,  0.5,  0.5, 0, 1,
                -0.5,  0.5,  0.5, 0, 0,
                 0.5,  0.5,  0.5, 1, 0,
                 0.5,  0.5, -0.5, 1, 1,
                -0.5,  0.5, -0.5, 0, 1,
                -0.5,  0.5, -0.5, 1, 1,
                 0.5,  0.5, -0.5, 0, 1,
                 0.5, -0.5, -0.5, 0, 0,
                -0.5, -0.5, -0.5, 1, 0,
                -0.5, -0.5, -0.5, 0, 0,
                 0.5, -0.5, -0.5, 1, 0,
                 0.5, -0.5,  0.5, 1, 1,
                -0.5, -0.5,  0.5, 0, 1,
                 0.5, -0.5,  0.5, 0, 0,
                 0.5, -0.5, -0.5, 1, 0,
                 0.5,  0.5, -0.5, 1, 1,
                 0.5,  0.5,  0.5, 0, 1,
                -0.5, -0.5, -0.5, 0, 0,
                -0.5, -0.5,  0.5, 1, 0,
                -0.5,  0.5,  0.5, 1, 1,
                -0.5,  0.5, -0.5, 0, 1
            ]
    };
    /*jshint white: true*/

    var vertexbuffer = graphicsDevice.createVertexBuffer(vertexbufferParameters);

    var semantics = graphicsDevice.createSemantics([graphicsDevice.SEMANTIC_POSITION, graphicsDevice.SEMANTIC_TEXCOORD]);

    /*jshint white: false*/
    var indexbufferParameters =
    {
        numIndices: 36,
        format: 'USHORT',
        dynamic: false,
        data: [
                 2,  0,  1,
                 3,  0,  2,
                 6,  4,  5,
                 7,  4,  6,
                10,  8,  9,
                11,  8, 10,
                14, 12, 13,
                15, 12, 14,
                18, 16, 17,
                19, 16, 18,
                22, 20, 21,
                23, 20, 22
            ]
    };


    if (graphicsDevice.beginFrame())
        {
            graphicsDevice.clear(clearColor, 1.0, 0);

            floor.render(graphicsDevice, camera);

            if (0 >= assetsToLoad)
            {
                graphicsDevice.setStream(vertexbuffer, semantics);

                graphicsDevice.setIndexBuffer(indexbuffer);

                graphicsDevice.setTechnique(technique3d);

                graphicsDevice.setTechniqueParameters(sharedTechniqueParameters);

                for (n = 0; n < numCubes; n += 1)
                {
                    boxBodies[n].calculateTransform(transform);
                    wvp = m43MulM44.call(mathDevice, transform, vp, wvp);
                    if (isVisibleBoxOrigin.call(mathDevice, cubeExtents, wvp))
                    {
                        // Use directly the active technique when just a single property changes
                        technique3d.worldViewProjection = wvp;
                        graphicsDevice.drawIndexed(primitive, numIndices);
                    }
                }

                if (!mouseForces.pickedBody)
                {
                    graphicsDevice.setTechnique(technique2d);

                    var screenWidth = graphicsDevice.width;
                    var screenHeight = graphicsDevice.height;
                    techniqueParameters2d['clipSpace'] =
                        v4Build.call(mathDevice, 2.0 / screenWidth, -2.0 / screenHeight, -1.0, 1.0);
                    graphicsDevice.setTechniqueParameters(techniqueParameters2d);

                    var writer = graphicsDevice.beginDraw(linePrim,
                                  4,
                                  cursorFormat,
                                  cursorSemantic);

                    if (writer)
                    {
                        var halfWidth = screenWidth * 0.5;
                        var halfHeight = screenHeight * 0.5;
                        writer([halfWidth - 10, halfHeight]);
                        writer([halfWidth + 10, halfHeight]);
                        writer([halfWidth, halfHeight - 10]);
                        writer([halfWidth, halfHeight + 10]);

                        graphicsDevice.endDraw(writer);
                    }
                }
            }

*/