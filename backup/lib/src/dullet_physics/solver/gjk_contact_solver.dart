part of dullet_physics;


//
// WebGL GJK Contact Solver
//WebGLGJKContactSolver
class WebGLGJKContactSolver
{
    static const int version = 1;

    Float32List simplex;
    double numVertices;
    Float32List closest;
    Float32List cachedCoords;
    Float32List tempCoords;

    removeVertex(int index)
    {
        this.numVertices -= 1;

        var simplex = this.simplex;
        var replace = (index * 9);
        var withv = (this.numVertices * 9);

        simplex[replace]     = simplex[withv];
        simplex[replace + 1] = simplex[withv + 1];
        simplex[replace + 2] = simplex[withv + 2];
        simplex[replace + 3] = simplex[withv + 3];
        simplex[replace + 4] = simplex[withv + 4];
        simplex[replace + 5] = simplex[withv + 5];
        simplex[replace + 6] = simplex[withv + 6];
        simplex[replace + 7] = simplex[withv + 7];
        simplex[replace + 8] = simplex[withv + 8];
    }

    reduceVertices(coords)
    {
        // NOTE: NOT USING EPSILON
        //
        // To avoid necessitating carrying 4 additional
        // boolean fields to mark coordinates as being
        // used or for deletion, the barycentric
        // coordinates are used to infer this property
        // instead. so strict equality with 0 is needed.
        if (this.numVertices >= 4 && coords[3] == 0) {
            this.numVertices -= 1;
        }

        var simplex = this.simplex;
        var withv;
        if (this.numVertices >= 3 && coords[2] == 0) {
            this.numVertices -= 1;
            withv = (this.numVertices * 9);
            simplex[18] = simplex[withv];
            simplex[19] = simplex[withv + 1];
            simplex[20] = simplex[withv + 2];
            simplex[21] = simplex[withv + 3];
            simplex[22] = simplex[withv + 4];
            simplex[23] = simplex[withv + 5];
            simplex[24] = simplex[withv + 6];
            simplex[25] = simplex[withv + 7];
            simplex[26] = simplex[withv + 8];
        }

        if (this.numVertices >= 2 && coords[1] == 0) {
            this.numVertices -= 1;
            withv = (this.numVertices * 9);
            simplex[9]  = simplex[withv];
            simplex[10] = simplex[withv + 1];
            simplex[11] = simplex[withv + 2];
            simplex[12] = simplex[withv + 3];
            simplex[13] = simplex[withv + 4];
            simplex[14] = simplex[withv + 5];
            simplex[15] = simplex[withv + 6];
            simplex[16] = simplex[withv + 7];
            simplex[17] = simplex[withv + 8];
        }

        if (this.numVertices >= 1 && coords[0] == 0)
        {
            this.numVertices -= 1;
            withv = (this.numVertices * 9);
            simplex[0] = simplex[withv];
            simplex[1] = simplex[withv + 1];
            simplex[2] = simplex[withv + 2];
            simplex[3] = simplex[withv + 3];
            simplex[4] = simplex[withv + 4];
            simplex[5] = simplex[withv + 5];
            simplex[6] = simplex[withv + 6];
            simplex[7] = simplex[withv + 7];
            simplex[8] = simplex[withv + 8];
        }
    }

    updateClosestPoints()
    {
        var numVertices = this.numVertices;
        if (numVertices == 0)
        {
            return false;
        }

        // ----------------------------------------
        // single vertex, only one candidate point.

        var simplex = this.simplex;
        var closest = this.closest;
        var i;

        if (numVertices == 1)
        {
            closest[0] = simplex[3];
            closest[1] = simplex[4];
            closest[2] = simplex[5];
            closest[3] = simplex[6];
            closest[4] = simplex[7];
            closest[5] = simplex[8];
            return true;
        }

        // ----------------------------------------
        // two vertices, find closest point on line

        var a0 = simplex[0];
        var a1 = simplex[1];
        var a2 = simplex[2];

        var b0 = simplex[9];
        var b1 = simplex[10];
        var b2 = simplex[11];

        if (numVertices == 2)
        {
            var w0 = (a0 - b0);
            var w1 = (a1 - b1);
            var w2 = (a2 - b2);

            var dot = ((a0 * w0) + (a1 * w1) + (a2 * w2));
            if (dot > 0)
            {
                var wlsq = ((w0 * w0) + (w1 * w1) + (w2 * w2));
                if (dot < wlsq)
                {
                    dot /= wlsq;
                    var dot1 = (1.0 - dot);

                    closest[0] = ((simplex[3] * dot1) + (simplex[12] * dot));
                    closest[1] = ((simplex[4] * dot1) + (simplex[13] * dot));
                    closest[2] = ((simplex[5] * dot1) + (simplex[14] * dot));
                    closest[3] = ((simplex[6] * dot1) + (simplex[15] * dot));
                    closest[4] = ((simplex[7] * dot1) + (simplex[16] * dot));
                    closest[5] = ((simplex[8] * dot1) + (simplex[17] * dot));

                    return true;
                }
                else
                {
                    this.removeVertex(0);
                }
            }
            else
            {
                this.removeVertex(1);
            }

            for (i = 0; i < 6; i += 1)
            {
                closest[i] = simplex[i + 3];
            }

            return true;
        }

        // ----------------------------------------
        // 3 vertices, find closest point in triangle

        var coords = this.cachedCoords;
        var alpha, beta, gamma;

        if (numVertices == 3)
        {
            this.closestPointTriangle(0, 9, 18, coords);
            this.reduceVertices(coords);

            alpha = coords[0];
            beta = coords[1];
            gamma = coords[2];

            closest[0] = ((alpha * simplex[3]) + (beta * simplex[12]) + (gamma * simplex[21]));
            closest[1] = ((alpha * simplex[4]) + (beta * simplex[13]) + (gamma * simplex[22]));
            closest[2] = ((alpha * simplex[5]) + (beta * simplex[14]) + (gamma * simplex[23]));
            closest[3] = ((alpha * simplex[6]) + (beta * simplex[15]) + (gamma * simplex[24]));
            closest[4] = ((alpha * simplex[7]) + (beta * simplex[16]) + (gamma * simplex[25]));
            closest[5] = ((alpha * simplex[8]) + (beta * simplex[17]) + (gamma * simplex[26]));

            return true;
        }

        // ----------------------------------------
        // 4 vertices, find closest point in tetrahedron

        if (numVertices == 4)
        {
            var outside = this.closestPointTetrahedron(coords);
            if (outside)
            {
                this.reduceVertices(coords);

                alpha = coords[0];
                beta = coords[1];
                gamma = coords[2];
                var delta = coords[3];

                closest[0] = ((alpha * simplex[3]) + (beta * simplex[12]) + (gamma * simplex[21]) + (delta * simplex[30]));
                closest[1] = ((alpha * simplex[4]) + (beta * simplex[13]) + (gamma * simplex[22]) + (delta * simplex[31]));
                closest[2] = ((alpha * simplex[5]) + (beta * simplex[14]) + (gamma * simplex[23]) + (delta * simplex[32]));
                closest[3] = ((alpha * simplex[6]) + (beta * simplex[15]) + (gamma * simplex[24]) + (delta * simplex[33]));
                closest[4] = ((alpha * simplex[7]) + (beta * simplex[16]) + (gamma * simplex[25]) + (delta * simplex[34]));
                closest[5] = ((alpha * simplex[8]) + (beta * simplex[17]) + (gamma * simplex[26]) + (delta * simplex[35]));

                return true;
            }
            else
            {
                return false;
            }
        }

        return false;
    }

    closestPointTetrahedron(coords)
    {
        var simplex = this.simplex;

        var a0 = simplex[0];
        var a1 = simplex[1];
        var a2 = simplex[2];

        var b0 = simplex[9];
        var b1 = simplex[10];
        var b2 = simplex[11];

        var c0 = simplex[18];
        var c1 = simplex[19];
        var c2 = simplex[20];

        var d0 = simplex[27];
        var d1 = simplex[28];
        var d2 = simplex[29];

        var ab0 = (b0 - a0);
        var ab1 = (b1 - a1);
        var ab2 = (b2 - a2);

        var ac0 = (c0 - a0);
        var ac1 = (c1 - a1);
        var ac2 = (c2 - a2);

        var ad0 = (d0 - a0);
        var ad1 = (d1 - a1);
        var ad2 = (d2 - a2);

        var bc0 = (c0 - b0);
        var bc1 = (c1 - b1);
        var bc2 = (c2 - b2);

        var bd0 = (d0 - b0);
        var bd1 = (d1 - b1);
        var bd2 = (d2 - b2);

        var n0, n1, n2, signD, signOrigin;

        n0 = ((ab1 * ac2) - (ab2 * ac1));
        n1 = ((ab2 * ac0) - (ab0 * ac2));
        n2 = ((ab0 * ac1) - (ab1 * ac0));
        signD = ((ad0 * n0) + (ad1 * n1) + (ad2 * n2));
        signOrigin = - ((a0 * n0) + (a1 * n1) + (a2 * n2));
        var sideABC = ((signOrigin * signD) <= 0);

        n0 = ((ac1 * ad2) - (ac2 * ad1));
        n1 = ((ac2 * ad0) - (ac0 * ad2));
        n2 = ((ac0 * ad1) - (ac1 * ad0));
        signD = ((ab0 * n0) + (ab1 * n1) + (ab2 * n2));
        signOrigin = - ((a0 * n0) + (a1 * n1) + (a2 * n2));
        var sideACD = ((signOrigin * signD) <= 0);

        n0 = ((ad1 * ab2) - (ad2 * ab1));
        n1 = ((ad2 * ab0) - (ad0 * ab2));
        n2 = ((ad0 * ab1) - (ad1 * ab0));
        signD = ((ac0 * n0) + (ac1 * n1) + (ac2 * n2));
        signOrigin = - ((a0 * n0) + (a1 * n1) + (a2 * n2));
        var sideADB = ((signOrigin * signD) <= 0);

        n0 = ((bd1 * bc2) - (bd2 * bc1));
        n1 = ((bd2 * bc0) - (bd0 * bc2));
        n2 = ((bd0 * bc1) - (bd1 * bc0));
        signD = ((ab0 * n0) + (ab1 * n1) + (ab2 * n2));
        signOrigin = ((b0 * n0) + (b1 * n1) + (b2 * n2));
        var sideBDC = ((signOrigin * signD) <= 0);

        coords[0] = coords[1] = coords[2] = coords[3] = 0.0;

        // inclusion
        if (!sideABC && !sideACD && !sideADB && !sideBDC)
        {
            return false;
        }

        var tempCoords = this.tempCoords;
        var minSqDist = Number.MAX_VALUE;
        var sqDist;

        if (sideABC)
        {
            sqDist = this.closestPointTriangle(0, 9, 18, tempCoords, true);
            if (sqDist < minSqDist)
            {
                minSqDist = sqDist;
                coords[0] = tempCoords[0];
                coords[1] = tempCoords[1];
                coords[2] = tempCoords[2];
                coords[3] = 0.0;
            }
        }

        if (sideACD)
        {
            sqDist = this.closestPointTriangle(0, 18, 27, tempCoords, true);
            if (sqDist < minSqDist)
            {
                minSqDist = sqDist;
                coords[0] = tempCoords[0];
                coords[1] = 0.0;
                coords[2] = tempCoords[1];
                coords[3] = tempCoords[2];
            }
        }

        if (sideADB)
        {
            sqDist = this.closestPointTriangle(0, 27, 9, tempCoords, true);
            if (sqDist < minSqDist)
            {
                minSqDist = sqDist;
                coords[0] = tempCoords[0];
                coords[1] = tempCoords[2];
                coords[2] = 0.0;
                coords[3] = tempCoords[1];
            }
        }

        if (sideBDC)
        {
            sqDist = this.closestPointTriangle(9, 27, 18, tempCoords, true);
            if (sqDist < minSqDist)
            {
                minSqDist = sqDist;
                coords[0] = 0.0;
                coords[1] = tempCoords[0];
                coords[2] = tempCoords[2];
                coords[3] = tempCoords[1];
            }
        }

        return true;
    }

    closestPointTriangle(a, b, c, coords, [bool computeDistance])
    {
        var simplex = this.simplex;

        var a0 = simplex[a];
        var a1 = simplex[a + 1];
        var a2 = simplex[a + 2];

        var b0 = simplex[b];
        var b1 = simplex[b + 1];
        var b2 = simplex[b + 2];

        var c0 = simplex[c];
        var c1 = simplex[c + 1];
        var c2 = simplex[c + 2];

        var ba0 = (a0 - b0);
        var ba1 = (a1 - b1);
        var ba2 = (a2 - b2);

        var ca0 = (a0 - c0);
        var ca1 = (a1 - c1);
        var ca2 = (a2 - c2);

        var dot1 = ((a0 * ba0) + (a1 * ba1) + (a2 * ba2));
        var dot2 = ((a0 * ca0) + (a1 * ca1) + (a2 * ca2));
        if (dot1 <= 0.0 && dot2 <= 0) // Origin in region outside of A
        {
            coords[0] = 1;
            coords[1] = coords[2] = 0;
            if (computeDistance)
            {
                return ((a0 * a0) + (a1 * a1) + (a2 * a2));
            }
            else
            {
                return null;
            }
        }

        var dot3 = ((b0 * ba0) + (b1 * ba1) + (b2 * ba2));
        var dot4 = ((b0 * ca0) + (b1 * ca1) + (b2 * ca2));
        if (dot3 >= 0.0 && dot4 <= dot3) // Origin in region outside of B
        {
            coords[1] = 1;
            coords[0] = coords[2] = 0;
            if (computeDistance)
            {
                return ((b0 * b0) + (b1 * b1) + (b2 * b2));
            }
            else
            {
                return null;
            }
        }

        var v;
        var d0, d1, d2;

        var vc = ((dot1 * dot4) - (dot3 * dot2));
        if (vc <= 0.0 && dot1 >= 0.0 && dot3 <= 0.0) // Origin in region outside A-B
        {
            v = (dot1 / (dot1 - dot3));
            coords[0] = (1 - v);
            coords[1] = v;
            coords[2] = 0;
            if (computeDistance)
            {
                d0 = (a0 - (v * ba0));
                d1 = (a1 - (v * ba1));
                d2 = (a2 - (v * ba2));
                return ((d0 * d0) + (d1 * d1) + (d2 * d2));
            }
            else
            {
                return null;
            }
        }

        var dot5 = ((c0 * ba0) + (c1 * ba1) + (c2 * ba2));
        var dot6 = ((c0 * ca0) + (c1 * ca1) + (c2 * ca2));
        if (dot6 >= 0.0 && dot5 <= dot6) // Origin in region outsiode of C
        {
            coords[0] = coords[1] = 0;
            coords[2] = 1;
            if (computeDistance)
            {
                return ((c0 * c0) + (c1 * c1) + (c2 * c2));
            }
            else
            {
                return null;
            }
        }

        var vb = ((dot5 * dot2) - (dot1 * dot6));
        if (vb <= 0.0 && dot2 >= 0.0 && dot6 <= 0.0) // Origin in region outside of A-C
        {
            v = (dot2 / (dot2 - dot6));
            coords[0] = (1 - v);
            coords[1] = 0;
            coords[2] = v;
            if (computeDistance)
            {
                d0 = (a0 - (v * ca0));
                d1 = (a1 - (v * ca1));
                d2 = (a2 - (v * ca2));
                return ((d0 * d0) + (d1 * d1) + (d2 * d2));
            }
            else
            {
                return null;
            }
        }

        var va = ((dot3 * dot6) - (dot5 * dot4));
        if (va <= 0.0 && (dot4 - dot3) >= 0.0 && (dot5 - dot6) >= 0.0) // Origin in region outside of B-C
        {
            v = ((dot4 - dot3) / ((dot4 - dot3) + (dot5 - dot6)));
            coords[0] = 0;
            coords[1] = (1 - v);
            coords[2] = v;
            if (computeDistance)
            {
                d0 = ((b0 * (1 - v)) + (c0 * v));
                d1 = ((b1 * (1 - v)) + (c1 * v));
                d2 = ((b2 * (1 - v)) + (c2 * v));
                return ((d0 * d0) + (d1 * d1) + (d2 * d2));
            }
            else
            {
                return null;
            }
        }

        // Origin contained in triangle region
        var denom = (1 / (va + vb + vc));
        v = (vb * denom);
        var w = (vc * denom);
        coords[0] = (1 - v - w);
        coords[1] = v;
        coords[2] = w;
        if (computeDistance)
        {
            d0 = (a0 - (ba0 * v) - (ca0 * w));
            d1 = (a1 - (ba1 * v) - (ca1 * w));
            d2 = (a2 - (ba2 * v) - (ca2 * w));
            return ((d0 * d0) + (d1 * d1) + (d2 * d2));
        }
        else
        {
            return null;
        }
    }

    //
    // cache having properties
    //   shapeA
    //   shapeB
    //   axis <-- to be mutated by this function
    //      'on' objectB.
    //   closestA <-- to be populated by this function
    //   closestB <-- to be populated by this function
    evaluate(cache, xformA, xformB)
    {
        var axis = cache.axis;
        var shapeA = cache.shapeA;
        var shapeB = cache.shapeB;

        // Reset GJK.
        this.numVertices = 0;
        var lastW0, lastW1, lastW2;
        lastW0 = lastW1 = lastW2 = Number.MAX_VALUE;

        var curIter = 0;
        var maxIter = 100;
        var seperated = false;

        var squaredDistance = Number.MAX_VALUE;

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

        var axis0 = axis[0];
        var axis1 = axis[1];
        var axis2 = axis[2];
        var axislsq;

        var supportA = cache.closestA;
        var supportB = cache.closestB;

        var closest = this.closest;
        var simplex = this.simplex;

        // Epsilon defined based on rough experimental result.
        var equalVertexThreshold = 1e-4;

        for (;;)
        {
            curIter += 1;

            // supportA = xformA * shapeA.localSupport ( - ixformA * axis)
            // supportB = xformB * shapeB.localSupport (   ixformB * axis)
            //this.m43InverseOrthonormalTransformVector(xformA, axis, supportA);
            //VMath.v3Neg(supportA, supportA);
            supportA[0] = -((A0 * axis0) + (A1 * axis1) + (A2 * axis2));
            supportA[1] = -((A3 * axis0) + (A4 * axis1) + (A5 * axis2));
            supportA[2] = -((A6 * axis0) + (A7 * axis1) + (A8 * axis2));

            //this.m43InverseOrthonormalTransformVector(xformB, axis, supportB);
            supportB[0] = ((B0 * axis0) + (B1 * axis1) + (B2 * axis2));
            supportB[1] = ((B3 * axis0) + (B4 * axis1) + (B5 * axis2));
            supportB[2] = ((B6 * axis0) + (B7 * axis1) + (B8 * axis2));

            shapeA.localSupportWithoutMargin(supportA, supportA);
            shapeB.localSupportWithoutMargin(supportB, supportB);

            //VMath.m43TransformPoint(xformA, supportA, supportA);
            var d0 = supportA[0];
            var d1 = supportA[1];
            var d2 = supportA[2];
            var sa0 = supportA[0] = ((A0 * d0) + (A3 * d1) + (A6 * d2) + A9);
            var sa1 = supportA[1] = ((A1 * d0) + (A4 * d1) + (A7 * d2) + A10);
            var sa2 = supportA[2] = ((A2 * d0) + (A5 * d1) + (A8 * d2) + A11);

            //VMath.m43TransformPoint(xformB, supportB, supportB);
            d0 = supportB[0];
            d1 = supportB[1];
            d2 = supportB[2];
            var sb0 = supportB[0] = ((B0 * d0) + (B3 * d1) + (B6 * d2) + B9);
            var sb1 = supportB[1] = ((B1 * d0) + (B4 * d1) + (B7 * d2) + B10);
            var sb2 = supportB[2] = ((B2 * d0) + (B5 * d1) + (B8 * d2) + B11);

            //VMath.v3Sub(supportA, supportB, w);
            var w0 = sa0 - sb0;
            var w1 = sa1 - sb1;
            var w2 = sa2 - sb2;

            // If point is already in simplex, then we have reached closest point to origin
            // and minkowski difference does not intersect origin.
            var inSimplex = false;
            var index = this.numVertices * 9;
            var i;
            for (i = 0; i < index; i += 9)
            {
                d0 = (w0 - simplex[i]);
                d1 = (w1 - simplex[i + 1]);
                d2 = (w2 - simplex[i + 2]);
                if (((d0 * d0) + (d1 * d1) + (d2 * d2)) < equalVertexThreshold)
                {
                    inSimplex = true;
                }
            }

            // Additionaly check against previously inserted vertex which may have been
            // removed and prevent endless oscillation.
            if (!inSimplex)
            {
                d0 = (w0 - lastW0);
                d1 = (w1 - lastW1);
                d2 = (w2 - lastW2);
                inSimplex = ((d0 * d0) + (d1 * d1) + (d2 * d2)) < equalVertexThreshold;
            }

            if (inSimplex)
            {
                seperated = true;
                break;
            }

            //delta = VMath.v3Dot(axis, w);
            var delta = (axis0 * w0) + (axis1 * w1) + (axis2 * w2);

            // Check that we are getting closer
            // If not (within epsilon) we are very roughly at closest point
            // and should terminate!
            //
            if ((squaredDistance - delta) <= (squaredDistance * WebGLPhysicsConfig.GJK_FRACTIONAL_THRESHOLD))
            {
                seperated = true;
                break;
            }

            // Add vertex to simplex.
            lastW0 = simplex[index] = w0;
            lastW1 = simplex[index + 1] = w1;
            lastW2 = simplex[index + 2] = w2;
            simplex[index + 3] = sa0;
            simplex[index + 4] = sa1;
            simplex[index + 5] = sa2;
            simplex[index + 6] = sb0;
            simplex[index + 7] = sb1;
            simplex[index + 8] = sb2;
            this.numVertices += 1;

            // If we cannot find a seperating axis
            // Then shapes are intersecting!
            if (!this.updateClosestPoints())
            {
                seperated = false;
                break;
            }

            d0 = (closest[0] - closest[3]);
            d1 = (closest[1] - closest[4]);
            d2 = (closest[2] - closest[5]);

            // If seperation distance is very (very) small
            // Then we assume shapes are intersecting.
            axislsq = ((d0 * d0) + (d1 * d1) + (d2 * d2));
            if (axislsq <= WebGLPhysicsConfig.GJK_EPA_DISTANCE_THRESHOLD)
            {
                seperated = true;
                break;
            }

            // Prepare for next iteration.
            //VMath.v3Copy(newaxis, axis);
            axis0 = d0;
            axis1 = d1;
            axis2 = d2;

            // Check that we are getting closer with true distances
            // If not, terminate!
            var previousSqDistance = squaredDistance;
            squaredDistance = axislsq;

            if ((previousSqDistance - squaredDistance) <= (WebGLPhysicsConfig.GJK_FRACTIONAL_THRESHOLD * previousSqDistance))
            {
                seperated = true;
                break;
            }

            if (curIter >= maxIter)
            {
                seperated = true;
                break;
            }

            // We already have a full simplex
            // Next iteration would add too many vertices
            // So we must be intersecting
            if (this.numVertices == 4)
            {
                break;
            }
        }

        // If we cannot normalise axis, then necessarigly
        // seperated = false.
        // We do not zero the axis, as it is still useful enough for EPA.
        axislsq = ((axis0 * axis0) + (axis1 * axis1) + (axis2 * axis2));
        if (axislsq < WebGLPhysicsConfig.DONT_NORMALIZE_THRESHOLD)
        {
            axis[0] = axis0;
            axis[1] = axis1;
            axis[2] = axis2;
            return undefined;
        }

        // Normalise axis whether GJK failed or succeeded:
        // Is useful information for futher investigations.
        var scale = 1 / Math.sqrt(axislsq);
        axis[0] = axis0 * scale;
        axis[1] = axis1 * scale;
        axis[2] = axis2 * scale;

        if (seperated)
        {
            // Get closest points in simplex.
            supportA[0] = closest[0];
            supportA[1] = closest[1];
            supportA[2] = closest[2];

            supportB[0] = closest[3];
            supportB[1] = closest[4];
            supportB[2] = closest[5];

            return Math.sqrt(squaredDistance);
        }
        else
        {
            return undefined;
        }
    }

    WebGLGJKContactSolver() {
        var solver = this;

        // current simplex with vertices W = P - Q, generated by points P and Q
        // [ W00 W01 W02 P01 P02 P03 Q01 Q02 Q03 ... ]
        solver.simplex = new Float32List(36);
        solver.numVertices = 0;

        // on update closest points defined by W = P - Q stored here.
        solver.closest = new Float32List(6);

        solver.cachedCoords = new Float32List(4);
        solver.tempCoords = new Float32List(4);

        //return solver;
    }
}

