part of dullet_physics;

//WebGLContactEPA.prototype.MAX_VERTICES = 64;
//WebGLContactEPA.prototype.MAX_FACES = 128;

//
// WebGLContactEPA
//
class WebGLContactEPA {
    static const int version = 1;

    static const int MAX_VERTICES = 64;// number;  // prototype
    static const int MAX_FACES = 128;// number;     // prototype

    Float32List vertex_store;

    var hull; // TODO: { root : null, count : 0 };
    var stock; // TODO: { root : null, count : 0 };
    var horizon; // TODO { cf : null, ff : null, numFaces : 0 };

    bind(faceA, edgeA, faceB, edgeB) {
        faceA.edge[edgeA] = edgeB;
        faceA.adjFace[edgeA] = faceB;
        faceB.edge[edgeB] = edgeA;
        faceB.adjFace[edgeB] = faceA;
    }

    append(list, face) {
        face.leaf0 = null;
        face.leaf1 = list.root;
        if (list.root) {
            list.root.leaf0 = face;
        }
        list.root = face;
        list.count += 1;
    }

    remove(list, face) {
        var leaf0 = face.leaf0;
        var leaf1 = face.leaf1;
        if (leaf1) {
            leaf1.leaf0 = leaf0;
        }
        if (leaf0) {
            leaf0.leaf1 = leaf1;
        }
        if (face == list.root) {
            list.root = leaf1;
        }
        list.count -= 1;
    }

    findBest() {
        var minFace = this.hull.root;
        var minDistance = minFace.distance * minFace.distance;
        var f;
        for (f = minFace.leaf1; f != null; f = f.leaf1) {
            var sqDistance = f.distance * f.distance;
            if (sqDistance < minDistance)
            {
                minFace = f;
                minDistance = sqDistance;
            }
        }

        return minFace;
    }

    getEdgeDistance(face, a, b) {
        var vertices = this.vertex_store;

        var a0 = vertices[a];
        var a1 = vertices[a + 1];
        var a2 = vertices[a + 2];

        var b0 = vertices[b];
        var b1 = vertices[b + 1];
        var b2 = vertices[b + 2];

        var ba0 = (b0 - a0);
        var ba1 = (b1 - a1);
        var ba2 = (b2 - a2);
        // outward facing edge normal on triangle plane

        var fn = face.normal;
        var fn0 = fn[0];
        var fn1 = fn[1];
        var fn2 = fn[2];

        var n0 = ((ba1 * fn2) - (ba2 * fn1));
        var n1 = ((ba2 * fn0) - (ba0 * fn2));
        var n2 = ((ba0 * fn1) - (ba1 * fn0));

        var dot = ((a0 * n0) + (a1 * n1) + (a2 * n2));
        if (dot <= 0) {
            //outside of edge A-B
            var lengthSqBA = ((ba0 * ba0) + (ba1 * ba1) + (ba2 * ba2));
            var dotA = ((a0 * ba0) + (a1 * ba1) + (a2 * ba2));
            var dotB = ((b0 * ba0) + (b1 * ba2) + (b2 * ba2));

            if (dotA >= 0) {
                //outside of vertex A
                return Math.sqrt((a0 * a0) + (a1 * a1) + (a2 * a2));
            } else if (dotB <= 0) {
                //outside of vertex B
                return Math.sqrt((b0 * b0) + (b1 * b1) + (b2 * b2));
            } else {
                var dotAB = ((a0 * b0) + (a1 * b1) + (a2 * b2));
                var dSq = (((a0 * a0) + (a1 * a1) + (a2 * a2)) * ((b0 * b0) + (b1 * b1) + (b2 * b2))) -
                          (dotAB * dotAB);
                return dSq >= 0 ? Math.sqrt(dSq / lengthSqBA) : 0;
            }
        } else {
            return null;
        }
    }

    buildNewFace(a, b, c, forced) {
        var face = this.stock.root;
        if (face == null)
        {
            return null;
        }

        face.pass = 0;
        face.vertex[0] = a;
        face.vertex[1] = b;
        face.vertex[2] = c;

        var vertices = this.vertex_store;

        var a0 = vertices[a];
        var a1 = vertices[a + 1];
        var a2 = vertices[a + 2];

        var b0 = vertices[b];
        var b1 = vertices[b + 1];
        var b2 = vertices[b + 2];

        var c0 = vertices[c];
        var c1 = vertices[c + 1];
        var c2 = vertices[c + 2];

        var ba0 = (b0 - a0);
        var ba1 = (b1 - a1);
        var ba2 = (b2 - a2);

        var ca0 = (c0 - a0);
        var ca1 = (c1 - a1);
        var ca2 = (c2 - a2);

        var fn = face.normal;
        var fn0 = fn[0] = ((ba1 * ca2) - (ba2 * ca1));
        var fn1 = fn[1] = ((ba2 * ca0) - (ba0 * ca2));
        var fn2 = fn[2] = ((ba0 * ca1) - (ba1 * ca0));
        var length = ((fn0 * fn0) + (fn1 * fn1) + (fn2 * fn2));

        if (length > WebGLPhysicsConfig.DONT_NORMALIZE_THRESHOLD)
        {
            face.distance = this.getEdgeDistance(face, a, b);

            if (face.distance == null)
            {
                face.distance = this.getEdgeDistance(face, b, c);
            }

            if (face.distance == null)
            {
                face.distance = this.getEdgeDistance(face, c, a);
            }

            var scale = 1 / Math.sqrt(length);
            if (face.distance == null)
            {
                // Origin must be closest to triangle plane.
                face.distance = ((a0 * fn0) + (a1 * fn1) + (a2 * fn2)) * scale;
            }

            // Epsilon based on rough experimental result.
            // Negative epsilon 'not' a typo!
            if (forced || (face.distance >= -1e-6))
            {
                fn[0] *= scale;
                fn[1] *= scale;
                fn[2] *= scale;

                // Success!
                this.remove(this.stock, face);
                this.append(this.hull, face);
                return face;
            }
        }

        return null;
    }

    expandFace(pass, w, face, edge, horizon) {
        if (face.pass != pass)
        {
            var fn = face.normal;
            var fn0 = fn[0];
            var fn1 = fn[1];
            var fn2 = fn[2];

            var vertices = this.vertex_store;
            var w0 = vertices[w];
            var w1 = vertices[w + 1];
            var w2 = vertices[w + 2];

            var edge1 = (edge + 1) % 3;
            // Epsilon based on rough experimental result
            // Negative epsilon 'not' a typo!
            if ((((fn0 * w0) + (fn1 * w1) + (fn2 * w2)) - face.distance) < -1e-6)
            {
                var newFace = this.buildNewFace(face.vertex[edge1], face.vertex[edge], w, false);
                if (newFace)
                {
                    this.bind(newFace, 0, face, edge);
                    if (horizon.cf)
                    {
                        this.bind(horizon.cf, 1, newFace, 2);
                    }
                    else
                    {
                        horizon.ff = newFace;
                    }
                    horizon.cf = newFace;
                    horizon.numFaces += 1;
                    return true;
                }
            }
            else
            {
                var edge2 = (edge + 2) % 3;
                face.pass = pass;
                if (this.expandFace(pass, w, face.adjFace[edge1], face.edge[edge1], horizon) &&
                    this.expandFace(pass, w, face.adjFace[edge2], face.edge[edge2], horizon))
                {
                    this.remove(this.hull, face);
                    this.append(this.stock, face);
                    return true;
                }
            }
        }

        return false;
    }

    //
    // cache having properties
    //   shapeA
    //   shapeB
    //   axis <-- to be mutated by this function
    //     'on' object B.
    //   closestA <-- to be populated by this function
    //   closestB <-- to be populated by this function
    evaluate(gjkSimplex, cache, xformA, xformB) {
        var shapeA = cache.shapeA;
        var shapeB = cache.shapeB;

        var hull = this.hull;
        var stock = this.stock;

        // Clean up after last evaluation
        while (hull.root)
        {
            var face = hull.root;
            this.remove(hull, face);
            this.append(stock, face);
        }

        // Orient simplex based on volume of tetrahedron
        var d0 = gjkSimplex[27];
        var d1 = gjkSimplex[28];
        var d2 = gjkSimplex[29];
        var ind0, ind1;

        var a0 = gjkSimplex[0] - d0;
        var a1 = gjkSimplex[1] - d1;
        var a2 = gjkSimplex[2] - d2;
        var b0 = gjkSimplex[9] - d0;
        var b1 = gjkSimplex[10] - d1;
        var b2 = gjkSimplex[11] - d2;
        var c0 = gjkSimplex[18] - d0;
        var c1 = gjkSimplex[19] - d1;
        var c2 = gjkSimplex[20] - d2;

        if (((a0 * ((b1 * c2) - (b2 * c1))) +
             (a1 * ((b2 * c0) - (b0 * c2))) +
             (a2 * ((b0 * c1) - (b1 * c0)))) < 0) {
            ind0 = 9;
            ind1 = 0;
        } else {
            ind0 = 0;
            ind1 = 9;
        }

        var vertices = this.vertex_store;
        var i;
        for (i = 0; i < 9; i += 1) {
            vertices[i] = gjkSimplex[ind0 + i];
            vertices[9 + i] = gjkSimplex[ind1 + i];
            vertices[18 + i] = gjkSimplex[18 + i];
            vertices[27 + i] = gjkSimplex[27 + i];
        }

        // Build initial convex hull
        var t0 = this.buildNewFace(0, 9, 18, true);
        var t1 = this.buildNewFace(9, 0, 27, true);
        var t2 = this.buildNewFace(18, 9, 27, true);
        var t3 = this.buildNewFace(0, 18, 27, true);

        var nextVertex = 36; //(4 * 9)

        if (hull.count != 4) {
            VMath.v3Build(gjkSimplex[3], gjkSimplex[4], gjkSimplex[5], cache.closestA);
            VMath.v3Build(gjkSimplex[6], gjkSimplex[7], gjkSimplex[8], cache.closestB);
            return 0;
        }

        var best = this.findBest();
        var pass = 0;
        var iterations = 0;

        this.bind(t0, 0, t1, 0);
        this.bind(t0, 1, t2, 0);
        this.bind(t0, 2, t3, 0);
        this.bind(t1, 1, t3, 2);
        this.bind(t1, 2, t2, 1);
        this.bind(t2, 2, t3, 1);

        // Cached for frequent access.
        var A0 = xformA[0];
        var A1 = xformA[1];
        var A2 = xformA[2];
        var A3 = xformA[3];
        var A4 = xformA[4];
        var A5 = xformA[5];
        var A6 = xformA[6];
        var A7 = xformA[7];
        var A8 = xformA[8];
        var A9 = xformA[9];
        var A10 = xformA[10];
        var A11 = xformA[11];

        var B0 = xformB[0];
        var B1 = xformB[1];
        var B2 = xformB[2];
        var B3 = xformB[3];
        var B4 = xformB[4];
        var B5 = xformB[5];
        var B6 = xformB[6];
        var B7 = xformB[7];
        var B8 = xformB[8];
        var B9 = xformB[9];
        var B10 = xformB[10];
        var B11 = xformB[11];

        var supportA = cache.closestA;
        var supportB = cache.closestB;

        var horizon = this.horizon;
        var bn, n0, n1, n2;

        for (; iterations < 100; iterations += 1)
        {
            if (nextVertex >= MAX_VERTICES * 9)
            {
                break;
            }

            // reset horizon
            horizon.cf = horizon.ff = null;
            horizon.numFaces = 0;

            // get vertex from pool
            var w = nextVertex;
            nextVertex += 9;

            pass += 1;
            best.pass = pass;

            // populate vertex with supports.
            bn = best.normal;
            n0 = bn[0];
            n1 = bn[1];
            n2 = bn[2];
            //WebGLPrivatePhysicsWorld.prototype.m43InverseOrthonormalTransformVector(xformA, best.normal, supportA);
            //WebGLPrivatePhysicsWorld.prototype.m43InverseOrthonormalTransformVector(xformB, best.normal, supportB);
            //VMath.v3Neg(supportB, supportB);
            supportA[0] = ((A0 * n0) + (A1 * n1) + (A2 * n2));
            supportA[1] = ((A3 * n0) + (A4 * n1) + (A5 * n2));
            supportA[2] = ((A6 * n0) + (A7 * n1) + (A8 * n2));

            supportB[0] = -((B0 * n0) + (B1 * n1) + (B2 * n2));
            supportB[1] = -((B3 * n0) + (B4 * n1) + (B5 * n2));
            supportB[2] = -((B6 * n0) + (B7 * n1) + (B8 * n2));

            shapeA.localSupportWithoutMargin(supportA, supportA);
            shapeB.localSupportWithoutMargin(supportB, supportB);

            //VMath.m43TransformPoint(xformA, supportA, supportA);
            d0 = supportA[0];
            d1 = supportA[1];
            d2 = supportA[2];
            a0 = ((A0 * d0) + (A3 * d1) + (A6 * d2) + A9);
            a1 = ((A1 * d0) + (A4 * d1) + (A7 * d2) + A10);
            a2 = ((A2 * d0) + (A5 * d1) + (A8 * d2) + A11);

            //VMath.m43TransformPoint(xformB, supportB, supportB);
            d0 = supportB[0];
            d1 = supportB[1];
            d2 = supportB[2];
            b0 = ((B0 * d0) + (B3 * d1) + (B6 * d2) + B9);
            b1 = ((B1 * d0) + (B4 * d1) + (B7 * d2) + B10);
            b2 = ((B2 * d0) + (B5 * d1) + (B8 * d2) + B11);

            var w0, w1, w2;
            vertices[w + 3] = a0;
            vertices[w + 4] = a1;
            vertices[w + 5] = a2;
            vertices[w + 6] = b0;
            vertices[w + 7] = b1;
            vertices[w + 8] = b2;
            vertices[w]     = w0 = (a0 - b0);
            vertices[w + 1] = w1 = (a1 - b1);
            vertices[w + 2] = w2 = (a2 - b2);

            // expand simplex
            var wDist = ((n0 * w0) + (n1 * w1) + (n2 * w2)) - best.distance;
            if (wDist > WebGLPhysicsConfig.GJK_EPA_DISTANCE_THRESHOLD)
            {
                var j;
                var valid = true;
                for (j = 0; (j < 3 && valid); j += 1)
                {
                    valid = valid && this.expandFace(pass, w, best.adjFace[j], best.edge[j], horizon);
                }

                if (valid && (horizon.numFaces >= 3))
                {
                    this.bind(horizon.cf, 1, horizon.ff, 2);
                    this.remove(hull, best);
                    this.append(stock, best);
                    best = this.findBest();
                }
                else
                {
                    break;
                }
            }
            else
            {
                break;
            }
        }

        bn = best.normal;
        n0 = bn[0];
        n1 = bn[1];
        n2 = bn[2];
        var bd = best.distance;

        // Projection of origin onto final face of simplex.
        var p0 = n0 * bd;
        var p1 = n1 * bd;
        var p2 = n2 * bd;

        c0 = best.vertex[0];
        c1 = best.vertex[1];
        c2 = best.vertex[2];

        var x0 = vertices[c0]     - p0;
        var x1 = vertices[c0 + 1] - p1;
        var x2 = vertices[c0 + 2] - p2;

        var y0 = vertices[c1]     - p0;
        var y1 = vertices[c1 + 1] - p1;
        var y2 = vertices[c1 + 2] - p2;

        var z0 = vertices[c2]     - p0;
        var z1 = vertices[c2 + 1] - p1;
        var z2 = vertices[c2 + 2] - p2;

        // Compute barycentric coordinates of origin's projection on face.
        d0 = ((y1 * z2) - (y2 * z1));
        d1 = ((y2 * z0) - (y0 * z2));
        d2 = ((y0 * z1) - (y1 * z0));
        var alpha = Math.sqrt((d0 * d0) + (d1 * d1) + (d2 * d2));

        d0 = ((z1 * x2) - (z2 * x1));
        d1 = ((z2 * x0) - (z0 * x2));
        d2 = ((z0 * x1) - (z1 * x0));
        var beta = Math.sqrt((d0 * d0) + (d1 * d1) + (d2 * d2));

        d0 = ((x1 * y2) - (x2 * y1));
        d1 = ((x2 * y0) - (x0 * y2));
        d2 = ((x0 * y1) - (x1 * y0));
        var gamma = Math.sqrt((d0 * d0) + (d1 * d1) + (d2 * d2));

        var scale = 1 / (alpha + beta + gamma);
        alpha *= scale;
        beta *= scale;
        gamma *= scale;

        // Interpolate for ideal support points.
        supportA[0] = supportA[1] = supportA[2] = 0;
        supportB[0] = supportB[1] = supportB[2] = 0;
        for (i = 0; i < 3; i += 1)
        {
            supportA[i] += (alpha * vertices[c0 + 3 + i]) + (beta * vertices[c1 + 3 + i]) + (gamma * vertices[c2 + 3 + i]);
            supportB[i] += (alpha * vertices[c0 + 6 + i]) + (beta * vertices[c1 + 6 + i]) + (gamma * vertices[c2 + 6 + i]);
        }

        var axis = cache.axis;
        axis[0] = -n0;
        axis[1] = -n1;
        axis[2] = -n2;
        return (-best.distance);
    }

    WebGLContactEPA() {
        var epa = this;
        var i;

        // populate vertex and face pools
        epa.vertex_store = new Float32List(epa.MAX_VERTICES * 9);

        var face_store = [];
        for (i = 0; i < epa.MAX_FACES; i += 1)
        {
            face_store[i] = {
                normal : VMath.v3BuildZero(),
                distance : 0,
                vertex : new Int16List(3),
                adjFace : [null, null, null],
                edge : new Int16List(3),

                leaf0 : null,
                leaf1 : null,
                pass : 0
            };
        }

        // initialise hull, stock and horizon
        epa.hull = {
            root : null,
            count : 0
        };

        epa.stock = {
            root : null,
            count : 0
        };

        epa.horizon = {
            cf : null,
            ff : null,
            numFaces : 0
        };

        // populat stock with all faces
        for (i = 0; i < epa.MAX_FACES; i += 1)
        {
            epa.append(epa.stock, face_store[epa.MAX_FACES - i - 1]);
        }

        //return epa;
    }
}

//WebGLContactEPA.prototype.MAX_VERTICES = 64;
//WebGLContactEPA.prototype.MAX_FACES = 128;

//
// WebGLPhysicsPublicContact
//
class WebGLPhysicsPublicContact {
    //Float32List localPointOnA;//  : any;  // v3 - getter
    //Float32List localPointOnB;//  : any;  // v3 - getter
    //Float32List worldNormalOnB;// : any; // v3 - getter
    //bool added;//          : boolean;
    //double distance;//       : number;

    var _private; // TODO: can this stuff just be turned into private
                   // members?


    Float32List get localPointOnA {
        var pr = this._private;
        return VMath.v3Build(pr[0], pr[1], pr[2]);
    }
    void set localPointOnA(Float32List point) {
        var pr = this._private;
        pr[0] = point[0];
        pr[1] = point[1];
        pr[2] = point[2];
    }
    Float32List get localPointOnB {
        var pr = this._private;
        return VMath.v3Build(pr[3], pr[4], pr[5]);
    }
    void set localPointOnB(Float32List point) {
        var pr = this._private;
        pr[3] = point[0];
        pr[4] = point[1];
        pr[5] = point[2];
    }

    Float32List get worldNormalOnB {
      var pr = this._private;
      return VMath.v3Build(pr[12], pr[13], pr[14]);
    }
    void set worldNormalOnB(Float32List normal) {
      var pr = this._private;
      pr[12] = normal[0];
      pr[13] = normal[1];
      pr[14] = normal[2];
    }

    bool get added => (0.0 < _private[51]);

    double get distance => _private[21];

    WebGLPhysicsPublicContact() {
      this._private = null;
      var p = this;
    }
}