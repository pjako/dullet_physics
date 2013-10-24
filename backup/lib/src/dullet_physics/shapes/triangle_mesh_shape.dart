part of dullet_physics;







//
// WebGLPhysicsTriangleMeshShape
//
class WebGLPhysicsTriangleMeshShape implements WebGLPhysicsShape
{
    static const int version = 1;
    final String type = "TRIANGLE_MESH";

    WebGLPhysicsPrivateShape _private;
    //type            : string; // prototype
    double margin;
    double radius;
    Vector3 halfExtents; // v3
    Vector3 inertia; // v3

    //_public         : WebGLPhysicsShape;

    WebGLPhysicsPrivateTriangleArray triangleArray;
    Vector3 center; // v3
    Vector3 collisionRadius; // v3

    rayTest(ray) {
        return this.triangleArray.rayTest(ray);
    }

    WebGLPhysicsTriangleMeshShape(params) {
        //var rett = new WebGLPhysicsShape();
        var t = this;
        //rett._private = t;
        //t._public = rett;

        var margin = (params.margin != null) ? params.margin : 0.04;
        var triangleArray = params.triangleArray;//params.triangleArray._private;

        var extents = triangleArray.extents;
        var e0 = extents[0];
        var e1 = extents[1];
        var e2 = extents[2];
        var e3 = extents[3];
        var e4 = extents[4];
        var e5 = extents[5];

        var h0 = ((0.5 * (e3 - e0)) + margin);
        var h1 = ((0.5 * (e4 - e1)) + margin);
        var h2 = ((0.5 * (e5 - e2)) + margin);
        var c0 = (0.5 * (e0 + e3));
        var c1 = (0.5 * (e1 + e4));
        var c2 = (0.5 * (e2 + e5));

        t.triangleArray = triangleArray;
        t.radius = Math.sqrt((h0 * h0) + (h1 * h1) + (h2 * h2));
        t.halfExtents = new Vector3(h0, h1, h2);
        if (c0 != 0.0 || c1 != 0.0 || c2 != 0.0) {
            t.center = new Vector3(c0, c1, c2);
        } else {
            t.center = null;
        }
        t.inertia = new Vector3(0.0, 0.0, 0.0);
        t.collisionRadius = margin;

        /*
        initShapeProperties(rett, "TRIANGLE_MESH");

        Object.defineProperty(rett, "triangleArray", {
            get : function shapeGetTriangleArray()
            {
                return this._private.triangleArray;
            },
            enumerable : true
        });

        return rett;
        */
    }
}

//WebGLPhysicsTriangleMeshShape.prototype.type = "TRIANGLE_MESH";
