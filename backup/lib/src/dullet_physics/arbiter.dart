part of dullet_physics;




//
// WebGLPhysicsArbiter
//
class WebGLPhysicsArbiter
{
    static const int version = 1;

    WebGLPhysicsPrivateBody objectA;
    WebGLPhysicsPrivateBody objectB;
    WebGLPhysicsShape shapeA;
    WebGLPhysicsShape shapeB;
    double friction;
    double restitution;
    List<WebGLPhysicsContact> contacts;
    List<WebGLPhysicsPublicContact> activeContacts;
    bool active;
    bool skipDiscreteCollisions;
    int contactFlags;
    bool trigger;

    WebGLPhysicsArbiter()
    {
        // Initialise all properties of arbiters
        // which will ever be used.

        // Object between which this contact patch is defined.
        // Precondition: objectA.id < objectB.id
        this.objectA = null;
        this.objectB = null;

        // Shapes between which this contact patch is defined.
        // Precondition: object#.shape = shape#
        this.shapeA = null;
        this.shapeB = null;

        // Pairwise friction and restitution values.
        this.friction    = 0.0;
        this.restitution = 0.0;

        // Set of contacts in patch.
        this.contacts = <WebGLPhysicsContact>[];

        // Set of contacts with negative distance for physics computaions.
        this.activeContacts = <WebGLPhysicsPublicContact>[];

        // Whether contact is active (As compared to being sleeping).
        this.active = true;

        // Flag used to ignore unneccesary discrete collision checks in post-continuous collisions.
        this.skipDiscreteCollisions = false;

        // Flags to signal processing of contact callbacks
        this.contactFlags = 0; // 1 - added, 2 - processed, 4 - removed

        // Flag to disable contact response
        this.trigger = false;
    }
    void insertContact(Float32List worldA, Float32List worldB, Float32List normal, double distance, bool concave) {
        var cn0 = normal[0];
        var cn1 = normal[1];
        var cn2 = normal[2];
        var clsq = ((cn0 * cn0) + (cn1 * cn1) + (cn2 * cn2));

        if (clsq < WebGLPhysicsConfig.DONT_NORMALIZE_THRESHOLD)
        {
            return;
        }

        var scale = 1 / Math.sqrt(clsq);
        cn0 *= scale;
        cn1 *= scale;
        cn2 *= scale;

        //WebGLPrivatePhysicsWorld.prototype.m43InverseOrthonormalTransformPoint(this.objectA.transform, worldA, c.localA);
        //WebGLPrivatePhysicsWorld.prototype.m43InverseOrthonormalTransformPoint(this.objectB.transform, worldB, c.localB);
        //var localA = c.localA;
        //var localB = c.localB;
        //var relA = c.relA;
        //var relB = c.relB;
        var objectA = this.objectA;
        var objectB = this.objectB;
        var xformA = objectA.transform;
        var xformB = objectB.transform;
        var r0 = worldA[0] - xformA[9];
        var r1 = worldA[1] - xformA[10];
        var r2 = worldA[2] - xformA[11];
        var ca0 = (xformA[0] * r0) + (xformA[1] * r1) + (xformA[2] * r2);
        var ca1 = (xformA[3] * r0) + (xformA[4] * r1) + (xformA[5] * r2);
        var ca2 = (xformA[6] * r0) + (xformA[7] * r1) + (xformA[8] * r2);
        var jAccN = 0;

        // Cull any contacts with different normal
        // Inherit accumulated impulse of nearby contact
        /*jshint bitwise: false*/
        var i = 0;
        var min = double.MAX_FINITE;
        var contacts = this.contacts;
        var d0, d1, d2;
        while (i < contacts.length) {
            WebGLPhysicsContact datad = contacts[i];
            // 0.9 chosen based on rough experimental results.
            if ((!concave) && ((cn0 * datad[12]) + (cn1 * datad[13]) + (cn2 * datad[14])) < 0.9)
            {
                contacts[i] = contacts[contacts.length - 1];
                contacts.removeLast();
                WebGLPhysicsContact.deallocate(datad);
                this.contactFlags |= 4; // removed
                continue;
            }

            //var dlocalA = d.localA;
            d0 = (ca0 - datad[0]);
            d1 = (ca1 - datad[1]);
            d2 = (ca2 - datad[2]);
            var sep = (d0 * d0) + (d1 * d1) + (d2 * d2);
            if (sep < WebGLPhysicsConfig.CONTACT_EQUAL_SQ_SEPERATION)
            {
                //c.jAccN = d.jAccN;
                jAccN = datad[40];
                contacts[i] = contacts[contacts.length - 1];
                contacts.removeLast();
                WebGLPhysicsContact.deallocate(datad);
                this.contactFlags |= 4; // removed
                min = sep;
                continue;
            }

            if (sep < WebGLPhysicsConfig.CONTACT_INHERIT_SQ_SEPERATION && sep < min)
            {
                //c.jAccN = d.jAccN;
                jAccN = datad[40];
                min = sep;
            }

            i += 1;
        }

        var data = WebGLPhysicsContact.allocate();

        data[0] = ca0;
        data[1] = ca1;
        data[2] = ca2;
        data[6] = r0;
        data[7] = r1;
        data[8] = r2;

        data[9] = r0 = worldB[0] - xformB[9];
        data[10] = r1 = worldB[1] - xformB[10];
        data[11] = r2 = worldB[2] - xformB[11];
        data[3] = (xformB[0] * r0) + (xformB[1] * r1) + (xformB[2] * r2);
        data[4] = (xformB[3] * r0) + (xformB[4] * r1) + (xformB[5] * r2);
        data[5] = (xformB[6] * r0) + (xformB[7] * r1) + (xformB[8] * r2);

        //c.distance = distance;
        data[21] = distance;

        // contact normal, normalised.
        //var basis = c.basis;
        data[12] = cn0;
        data[13] = cn1;
        data[14] = cn2;

        // contact tangent.
        var ct0, /*ct1,*/ ct2;
        clsq = ((cn0 * cn0) + (cn2 * cn2));
        if (clsq < WebGLPhysicsConfig.DONT_NORMALIZE_THRESHOLD)
        {
            data[15] = ct0 = 1.0;
            data[16] = /*ct1 =*/ 0.0;
            data[17] = ct2 = 0.0;
        }
        else
        {
            scale = 1 / Math.sqrt(clsq);
            data[15] = ct0 = (-cn2 * scale);
            data[16] = /*ct1 =*/ 0.0;
            data[17] = ct2 = (cn0 * scale);
        }

        // contact bitangent
        data[18] = ((cn1 * ct2) /*- (cn2 * ct1)*/);
        data[19] = ((cn2 * ct0) - (cn0 * ct2));
        data[20] = (/*(cn0 * ct1)*/ - (cn1 * ct0));

        data[40] = jAccN;

        var contactCallbacks, publicContact;
        contactCallbacks = objectA.contactCallbacks;
        if (null != contactCallbacks && 0 != (contactCallbacks.mask & objectB.group))
        {
            if (contactCallbacks.onPreSolveContact)
            {
                publicContact = WebGLPhysicsContact.publicContacts[0];
                publicContact._private = data;
                contactCallbacks.onPreSolveContact(objectA._public, objectB._public, publicContact);
            }
            if (!contactCallbacks.added && contactCallbacks.deferred)
            {
                contactCallbacks.added = true;
                objectA.world.contactCallbackObjects.push(objectA);
            }
            if (contactCallbacks.trigger)
            {
                this.trigger = true;
                objectA.sweepFrozen = false;
                objectB.sweepFrozen = false;
            }
        }
        contactCallbacks = objectB.contactCallbacks;
        if (null != contactCallbacks && 0 != (contactCallbacks.mask & objectA.group))
        {
            if (contactCallbacks.onPreSolveContact)
            {
                publicContact = WebGLPhysicsContact.publicContacts[0];
                publicContact._private = data;
                contactCallbacks.onPreSolveContact(objectA._public, objectB._public, publicContact);
            }
            if (!contactCallbacks.added && contactCallbacks.deferred)
            {
                contactCallbacks.added = true;
                objectB.world.contactCallbackObjects.push(objectB);
            }
            if (contactCallbacks.trigger)
            {
                this.trigger = true;
                objectA.sweepFrozen = false;
                objectB.sweepFrozen = false;
            }
        }

        this.contactFlags |= 1; // added

        contacts.add(data);

        if (contacts.length == 4)
        {
            // Discard one contact, so that remaining 3 have maximum area, and contain deepest contact
            // Find deepest.
            var minDistance = contacts[0][21];
            var minimum = 0;
            for (i = 1; i < 4; i += 1)
            {
                data = contacts[i];
                if (data[21] < minDistance)
                {
                    minDistance = data[21];
                    minimum = i;
                }
            }

            var discard;
            var maxArea = -double.MAX_FINITE;//-Number.MAX_VALUE;

            var con0 = contacts[0];
            var con1 = contacts[1];
            var con2 = contacts[2];
            var con3 = contacts[3];

            // World coordinates of contact points (Scaled and translated, but does not matter).
            var a0 = con0[6] + con0[9];
            var a1 = con0[7] + con0[10];
            var a2 = con0[8] + con0[11];

            var b0 = con1[6] + con1[9];
            var b1 = con1[7] + con1[10];
            var b2 = con1[8] + con1[11];

            var c0 = con2[6] + con2[9];
            var c1 = con2[7] + con2[10];
            var c2 = con2[8] + con2[11];

            d0 = con3[6] + con3[9];
            d1 = con3[7] + con3[10];
            d2 = con3[8] + con3[11];

            var ab0 = (b0 - a0);
            var ab1 = (b1 - a1);
            var ab2 = (b2 - a2);

            var ac0 = (c0 - a0);
            var ac1 = (c1 - a1);
            var ac2 = (c2 - a2);

            var ad0 = (d0 - a0);
            var ad1 = (d1 - a1);
            var ad2 = (d2 - a2);

            var n0, n1, n2;
            var area;

            // Area discarding contact 1
            if (minimum != 1)
            {
                n0 = ((ac1 * ad2) - (ac2 * ad1));
                n1 = ((ac2 * ad0) - (ac0 * ad2));
                n2 = ((ac0 * ad1) - (ac1 * ad0));
                area = (n0 * n0) + (n1 * n1) + (n2 * n2);
                if (area > maxArea)
                {
                    maxArea = area;
                    discard = 1;
                }
            }

            // Area discarding contact 2
            if (minimum != 2)
            {
                n0 = ((ab1 * ad2) - (ab2 * ad1));
                n1 = ((ab2 * ad0) - (ab0 * ad2));
                n2 = ((ab0 * ad1) - (ab1 * ad0));
                area = (n0 * n0) + (n1 * n1) + (n2 * n2);
                if (area > maxArea)
                {
                    maxArea = area;
                    discard = 2;
                }
            }

            // Area discarding contact 3
            if (minimum != 3)
            {
                n0 = ((ab1 * ac2) - (ab2 * ac1));
                n1 = ((ab2 * ac0) - (ab0 * ac2));
                n2 = ((ab0 * ac1) - (ab1 * ac0));
                area = (n0 * n0) + (n1 * n1) + (n2 * n2);
                if (area > maxArea)
                {
                    maxArea = area;
                    discard = 3;
                }
            }

            // Area discarding contact 0
            if (minimum != 0)
            {
                var bc0 = (c0 - b0);
                var bc1 = (c1 - b1);
                var bc2 = (c2 - b2);

                var bd0 = (d0 - b0);
                var bd1 = (d1 - b1);
                var bd2 = (d2 - b2);

                n0 = ((bc1 * bd2) - (bc2 * bd1));
                n1 = ((bc2 * bd0) - (bc0 * bd2));
                n2 = ((bc0 * bd1) - (bc1 * bd0));
                area = (n0 * n0) + (n1 * n1) + (n2 * n2);
                if (area > maxArea)
                {
                    maxArea = area;
                    discard = 0;
                }
            }

            data = contacts[discard];
            contacts[discard] = contacts[3];
            contacts.removeLast();
            WebGLPhysicsContact.deallocate(data);
            this.contactFlags |= 4; // removed
        }
        /*jshint bitwise: true*/
    }

    bool refreshContacts()
    {
        var contacts = this.contacts;
        var objectA = this.objectA;
        var objectB = this.objectB;

        var xformA = objectA.transform;
        var xformB = objectB.transform;

        // Cached for use throughout method.
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

        var data;
        var i = 0;
        while (i < contacts.length)
        {
            data = contacts[i];

            //VMath.m43TransformVector(this.objectA.transform, c.localA, c.relA);
            var v0 = data[0];
            var v1 = data[1];
            var v2 = data[2];
            var ra0 = data[6] = ((A0 * v0) + (A3 * v1) + (A6 * v2));
            var ra1 = data[7] = ((A1 * v0) + (A4 * v1) + (A7 * v2));
            var ra2 = data[8] = ((A2 * v0) + (A5 * v1) + (A8 * v2));

            //VMath.m43TransformVector(this.objectB.transform, c.localB, c.relB);
            v0 = data[3];
            v1 = data[4];
            v2 = data[5];
            var rb0 = data[9] = ((B0 * v0) + (B3 * v1) + (B6 * v2));
            var rb1 = data[10] = ((B1 * v0) + (B4 * v1) + (B7 * v2));
            var rb2 = data[11] = ((B2 * v0) + (B5 * v1) + (B8 * v2));

            // contact seperation.
            v0 = (ra0 + A9) - (rb0 + B9);
            v1 = (ra1 + A10) - (rb1 + B10);
            v2 = (ra2 + A11) - (rb2 + B11);

            //var basis = c.basis;
            var n0 = data[12];
            var n1 = data[13];
            var n2 = data[14];

            //c.distance = VMath.v3Dot(c.normal, seperation);
            var sep = data[21] = ((n0 * v0) + (n1 * v1) + (n2 * v2));
            if (sep > WebGLPhysicsConfig.CONTACT_MAX_Y_SEPERATION)
            {
                contacts[i] = contacts[contacts.length - 1];
                contacts.removeLast();
                WebGLPhysicsContact.deallocate(data);
                this.contactFlags |= 4; // removed
                continue;
            }

            //VMath.v3AddScalarMul(seperation, c.normal, -c.distance, seperation);
            v0 -= (n0 * sep);
            v1 -= (n1 * sep);
            v2 -= (n2 * sep);

            if (((v0 * v0) + (v1 * v1) + (v2 * v2)) > WebGLPhysicsConfig.CONTACT_MAX_SQ_XZ_SEPERATION)
            {
                contacts[i] = contacts[contacts.length - 1];
                contacts.removeLast();
                WebGLPhysicsContact.deallocate(data);
                this.contactFlags |= 4; // removed
                continue;
            }

            i += 1;
        }

        this.contactFlags |= 2; // processed

        return (contacts.length == 0);
    }

    void preStep(double timeStepRatio, double timeStep)
    {
        if (this.trigger)
        {
            this.activeContacts.length = 0;
            return;
        }

        var objectA = this.objectA;
        var objectB = this.objectB;
        var mass_sum = objectA.inverseMass + objectB.inverseMass;

        var velA = objectA.velocity;
        var velB = objectB.velocity;

        // cached for frequent access.
        var I = objectA.inverseInertia;
        var A0 = I[0];
        var A1 = I[1];
        var A2 = I[2];
        var A3 = I[3];
        var A4 = I[4];
        var A5 = I[5];
        var A6 = I[6];
        var A7 = I[7];
        var A8 = I[8];

        I = objectB.inverseInertia;
        var B0 = I[0];
        var B1 = I[1];
        var B2 = I[2];
        var B3 = I[3];
        var B4 = I[4];
        var B5 = I[5];
        var B6 = I[6];
        var B7 = I[7];
        var B8 = I[8];

        var activeContacts = this.activeContacts;
        activeContacts.length = 0;

        // TOOD: REMOVE the <any> casts.  objectA appears to be a
        // WebGLPhysicsCollisionObject.  Does that have a
        // 'collisionObject' property?

        var baum = (objectA.collisionObject || objectB.collisionObject) ?
                      WebGLPhysicsConfig.CONTACT_STATIC_BAUMGRAUTE :
                      WebGLPhysicsConfig.CONTACT_BAUMGRAUTE;

        var contacts = this.contacts;
        var i;
        var limit = contacts.length;
        for (i = 0; i < limit; i += 1)
        {
            var data = contacts[i];
            if (data[21] > 0)
            {
                continue;
            }

            // TODO: remove this cast and fix the type error

            activeContacts[activeContacts.length] = data as List<WebGLPhysicsPublicContact>;

            // cacheing friction impulses between steps
            // caused them to fight eachother instead of stabalising at 0.
            data[41] = data[42] = 0.0;

            var ca0, ca1, ca2;
            var cb0, cb1, cb2;

            //var basis = c.basis;
            var n0 = data[12];
            var n1 = data[13];
            var n2 = data[14];

            var ra0 = data[6];
            var ra1 = data[7];
            var ra2 = data[8];

            var rb0 = data[9];
            var rb1 = data[10];
            var rb2 = data[11];

            //var jac = c.jac;
            var k0, k1, k2;

            // Compute effective mass and jacobian of penetration constraint.
            var kN = mass_sum;
            //crossA = VMath.v3Cross(c.relA, c.normal);
            ca0 = ((ra1 * n2) - (ra2 * n1));
            ca1 = ((ra2 * n0) - (ra0 * n2));
            ca2 = ((ra0 * n1) - (ra1 * n0));
            //c.nCrossA = VMath.m33Transform(objectA.inverseInertia, crossA);
            data[22] = k0 = ((A0 * ca0) + (A3 * ca1) + (A6 * ca2));
            data[23] = k1 = ((A1 * ca0) + (A4 * ca1) + (A7 * ca2));
            data[24] = k2 = ((A2 * ca0) + (A5 * ca1) + (A8 * ca2));
            kN += ((ca0 * k0) + (ca1 * k1) + (ca2 * k2));

            //crossB = VMbth.v3Cross(c.relB, c.normal);
            cb0 = ((rb1 * n2) - (rb2 * n1));
            cb1 = ((rb2 * n0) - (rb0 * n2));
            cb2 = ((rb0 * n1) - (rb1 * n0));
            //c.nCrossB = VMbth.m33Trbnsform(objectB.inverseInertib, crossB);
            data[25] = k0 = -((B0 * cb0) + (B3 * cb1) + (B6 * cb2));
            data[26] = k1 = -((B1 * cb0) + (B4 * cb1) + (B7 * cb2));
            data[27] = k2 = -((B2 * cb0) + (B5 * cb1) + (B8 * cb2));
            kN -= ((cb0 * k0) + (cb1 * k1) + (cb2 * k2));

            data[45] = 1 / kN;

            // Compute positional bias for baumgraute stabalisation#
            data[43] = baum * Math.min(0, data[21] + WebGLPhysicsConfig.CONTACT_SLOP) / timeStep;
            data[44] = 0;

            // Compute velocity at contact
            // var vel = VMath.v3Sub(velA, velB);
            var vel0 = (velA[0] - velB[0]);
            var vel1 = (velA[1] - velB[1]);
            var vel2 = (velA[2] - velB[2]);
            // vel += VMath.v3Cross(angA, c.relA);
            vel0 += ((velA[4] * ra2) - (velA[5] * ra1));
            vel1 += ((velA[5] * ra0) - (velA[3] * ra2));
            vel2 += ((velA[3] * ra1) - (velA[4] * ra0));
            // vel -= VMath.v3Cross(velB, c.relB);
            vel0 -= ((velB[4] * rb2) - (velB[5] * rb1));
            vel1 -= ((velB[5] * rb0) - (velB[3] * rb2));
            vel2 -= ((velB[3] * rb1) - (velB[4] * rb0));

            // Compute bounce bias.
            //c.bounce = VMath.v3Dot(vel, c.normal) * this.restitution;
            var bounce = ((vel0 * n0) + (vel1 * n1) + (vel2 * n2)) * this.restitution;
            // Epsilon based on experimental result
            if (bounce * bounce < 1e-2)
            {
                bounce = 0;
            }
            data[50] = bounce;

            // Compute effective mass and jacobian of friction constraint.
            var kU = mass_sum;
            n0 = data[15];
            n1 = data[16];
            n2 = data[17];

            //crossA = VMath.v3Cross(c.relA, c.tangent);
            ca0 = ((ra1 * n2) - (ra2 * n1));
            ca1 = ((ra2 * n0) - (ra0 * n2));
            ca2 = ((ra0 * n1) - (ra1 * n0));
            //c.uCrossA = VMath.m33Transform(objecnA.inverseInertia, crossA);
            data[28] = k0 = ((A0 * ca0) + (A3 * ca1) + (A6 * ca2));
            data[29] = k1 = ((A1 * ca0) + (A4 * ca1) + (A7 * ca2));
            data[30] = k2 = ((A2 * ca0) + (A5 * ca1) + (A8 * ca2));
            kU += ((ca0 * k0) + (ca1 * k1) + (ca2 * k2));

            //crossB = VMbth.v3Cross(c.relB, c.tangent);
            cb0 = ((rb1 * n2) - (rb2 * n1));
            cb1 = ((rb2 * n0) - (rb0 * n2));
            cb2 = ((rb0 * n1) - (rb1 * n0));
            //c.uCrossB = VMbth.m33Trbnsform(objecnB.inverseInertib, crossB);
            data[31] = k0 = -((B0 * cb0) + (B3 * cb1) + (B6 * cb2));
            data[32] = k1 = -((B1 * cb0) + (B4 * cb1) + (B7 * cb2));
            data[33] = k2 = -((B2 * cb0) + (B5 * cb1) + (B8 * cb2));
            kU -= ((cb0 * k0) + (cb1 * k1) + (cb2 * k2));

            var kV = mass_sum;
            n0 = data[18];
            n1 = data[19];
            n2 = data[20];

            //crossA = VMath.v3Cross(c.relA, c.bitangent);
            ca0 = ((ra1 * n2) - (ra2 * n1));
            ca1 = ((ra2 * n0) - (ra0 * n2));
            ca2 = ((ra0 * n1) - (ra1 * n0));
            //c.vCrossA = VMath.m33Transform(objecnA.inverseInertia, crossA);
            data[34] = k0 = ((A0 * ca0) + (A3 * ca1) + (A6 * ca2));
            data[35] = k1 = ((A1 * ca0) + (A4 * ca1) + (A7 * ca2));
            data[36] = k2 = ((A2 * ca0) + (A5 * ca1) + (A8 * ca2));
            kV += ((ca0 * k0) + (ca1 * k1) + (ca2 * k2));

            //crossB = VMbth.v3Cross(c.relB, c.bitangent);
            cb0 = ((rb1 * n2) - (rb2 * n1));
            cb1 = ((rb2 * n0) - (rb0 * n2));
            cb2 = ((rb0 * n1) - (rb1 * n0));
            //c.vCrossB = VMbth.m33Trbnsform(objecnB.inverseInertib, crossB);
            data[37] = k0 = -((B0 * cb0) + (B3 * cb1) + (B6 * cb2));
            data[38] = k1 = -((B1 * cb0) + (B4 * cb1) + (B7 * cb2));
            data[39] = k2 = -((B2 * cb0) + (B5 * cb1) + (B8 * cb2));
            kV -= ((cb0 * k0) + (cb1 * k1) + (cb2 * k2));

            var kUV = 0.0;
            kUV += ((ca0 * data[28]) + (ca1 * data[29]) + (ca2 * data[30]));
            kUV -= ((cb0 * data[31]) + (cb1 * data[32]) + (cb2 * data[33]));

            var idet = 1 / (kU * kV - kUV * kUV);
            data[46] = kV * idet;
            data[47] = -kUV * idet;
            data[48] = kU * idet;

            // scale cached impulse for change in time step
            data[40] *= timeStepRatio;
        }
    }

    void applyCachedImpulses() {
        if (this.trigger)
        {
            return;
        }

        var objectA = this.objectA;
        var objectB = this.objectB;

        var velA = objectA.velocity;
        var velB = objectB.velocity;

        var imA = objectA.inverseMass;
        var imB = objectB.inverseMass;

        var contacts = this.activeContacts;
        var i;
        for (i = 0; i < contacts.length; i += 1)
        {
            var data = contacts[i];

            var jn = data[40];
            var n0 = (data[12] * jn);
            var n1 = (data[13] * jn);
            var n2 = (data[14] * jn);

            velA[0] += (n0 * imA);
            velA[1] += (n1 * imA);
            velA[2] += (n2 * imA);

            velB[0] -= (n0 * imB);
            velB[1] -= (n1 * imB);
            velB[2] -= (n2 * imB);

            velA[3] += (data[22] * jn);
            velA[4] += (data[23] * jn);
            velA[5] += (data[24] * jn);

            velB[3] += (data[25] * jn);
            velB[4] += (data[26] * jn);
            velB[5] += (data[27] * jn);
        }
    }

    void computeAndApplyBiasImpulses() {
        if (this.trigger)
        {
            return;
        }

        var objectA = this.objectA;
        var objectB = this.objectB;

        // Set velocities to local vars.
        var vec = objectA.velocity;
        var va0 = vec[6];
        var va1 = vec[7];
        var va2 = vec[8];
        var wa0 = vec[9];
        var wa1 = vec[10];
        var wa2 = vec[11];

        vec = objectB.velocity;
        var vb0 = vec[6];
        var vb1 = vec[7];
        var vb2 = vec[8];
        var wb0 = vec[9];
        var wb1 = vec[10];
        var wb2 = vec[11];

        var imA = objectA.inverseMass;
        var imB = objectB.inverseMass;

        var contacts = this.activeContacts;
        var limit = contacts.length;
        var data;
        var i;
        for (i = 0; i < limit; i += 1)
        {
            data = contacts[i];

            var n0 = data[12];
            var n1 = data[13];
            var n2 = data[14];

            var ra0 = data[6];
            var ra1 = data[7];
            var ra2 = data[8];

            var rb0 = data[9];
            var rb1 = data[10];
            var rb2 = data[11];

            // Velocity normal impulse.
            var j1 = data[45] * (
                  n0 * ((vb0 + ((wb1 * rb2) - (wb2 * rb1))) - (va0 + ((wa1 * ra2) - (wa2 * ra1)))) +
                  n1 * ((vb1 + ((wb2 * rb0) - (wb0 * rb2))) - (va1 + ((wa2 * ra0) - (wa0 * ra2)))) +
                  n2 * ((vb2 + ((wb0 * rb1) - (wb1 * rb0))) - (va2 + ((wa0 * ra1) - (wa1 * ra0)))) -
                  data[43]);

            // Accumulate and clamp.
            var jOld1 = data[44];
            var cjAcc1 = jOld1 + j1;
            if (cjAcc1 < 0)
            {
                cjAcc1 = 0.0;
            }
            j1 = cjAcc1 - jOld1;
            data[44] = cjAcc1;

            // Apply normal impulse.
            n0 *= j1;
            n1 *= j1;
            n2 *= j1;

            va0 += (n0 * imA);
            va1 += (n1 * imA);
            va2 += (n2 * imA);

            vb0 -= (n0 * imB);
            vb1 -= (n1 * imB);
            vb2 -= (n2 * imB);

            wa0 += (data[22] * j1);
            wa1 += (data[23] * j1);
            wa2 += (data[24] * j1);

            wb0 += (data[25] * j1);
            wb1 += (data[26] * j1);
            wb2 += (data[27] * j1);
        }

        // Set local vars to velocities.
        vec = objectA.velocity;
        vec[6] = va0;
        vec[7] = va1;
        vec[8] = va2;
        vec[9] = wa0;
        vec[10] = wa1;
        vec[11] = wa2;

        vec = objectB.velocity;
        vec[6] = vb0;
        vec[7] = vb1;
        vec[8] = vb2;
        vec[9] = wb0;
        vec[10] = wb1;
        vec[11] = wb2;
    }

    void computeAndApplyImpulses()
    {
        if (this.trigger)
        {
            return;
        }

        var objectA = this.objectA;
        var objectB = this.objectB;

        // Set velocities to local vars.
        var vec = objectA.velocity;
        var va0 = vec[0];
        var va1 = vec[1];
        var va2 = vec[2];
        var wa0 = vec[3];
        var wa1 = vec[4];
        var wa2 = vec[5];

        vec = objectB.velocity;
        var vb0 = vec[0];
        var vb1 = vec[1];
        var vb2 = vec[2];
        var wb0 = vec[3];
        var wb1 = vec[4];
        var wb2 = vec[5];

        var imA = objectA.inverseMass;
        var imB = objectB.inverseMass;

        var friction = this.friction;

        var contacts = this.activeContacts;
        var limit = contacts.length;
        var data;
        var i;
        for (i = 0; i < limit; i += 1)
        {
            data = contacts[i];

            var n0 = data[12];
            var n1 = data[13];
            var n2 = data[14];
            var u0 = data[15];
            var u1 = data[16];
            var u2 = data[17];
            var v0 = data[18];
            var v1 = data[19];
            var v2 = data[20];

            var ra0 = data[6];
            var ra1 = data[7];
            var ra2 = data[8];

            var rb0 = data[9];
            var rb1 = data[10];
            var rb2 = data[11];

            // Velocity normal impulse.
            var j1 = data[45] * (
                  n0 * ((vb0 + ((wb1 * rb2) - (wb2 * rb1))) - (va0 + ((wa1 * ra2) - (wa2 * ra1)))) +
                  n1 * ((vb1 + ((wb2 * rb0) - (wb0 * rb2))) - (va1 + ((wa2 * ra0) - (wa0 * ra2)))) +
                  n2 * ((vb2 + ((wb0 * rb1) - (wb1 * rb0))) - (va2 + ((wa0 * ra1) - (wa1 * ra0)))) -
                  data[50]);

            // Accumulate and clamp.
            var jOld1 = data[40];
            var cjAcc1 = jOld1 + j1;
            if (cjAcc1 < 0)
            {
                cjAcc1 = 0.0;
                //j1 = cjAcc1 - jOld1;
                j1 = -jOld1;
            }
            data[40] = cjAcc1;

            // Apply normal impulse.
            n0 *= j1;
            n1 *= j1;
            n2 *= j1;

            va0 += (n0 * imA);
            va1 += (n1 * imA);
            va2 += (n2 * imA);

            vb0 -= (n0 * imB);
            vb1 -= (n1 * imB);
            vb2 -= (n2 * imB);

            wa0 += (data[22] * j1);
            wa1 += (data[23] * j1);
            wa2 += (data[24] * j1);

            wb0 += (data[25] * j1);
            wb1 += (data[26] * j1);
            wb2 += (data[27] * j1);

            // Relative velocity at contact point.
            n0 = (vb0 - va0) + ((wb1 * rb2) - (wb2 * rb1)) - ((wa1 * ra2) - (wa2 * ra1));
            n1 = (vb1 - va1) + ((wb2 * rb0) - (wb0 * rb2)) - ((wa2 * ra0) - (wa0 * ra2));
            n2 = (vb2 - va2) + ((wb0 * rb1) - (wb1 * rb0)) - ((wa0 * ra1) - (wa1 * ra0));

            // Friction tangent and bitangent constraint space impulses.
            var lambdau = ((u0 * n0) + (u1 * n1) + (u2 * n2));
            var lambdav = ((v0 * n0) + (v1 * n1) + (v2 * n2));

            // Transform by inverse mass matrix.
            j1 = lambdau * data[46] + lambdav * data[47];
            var j2 = lambdau * data[47] + lambdav * data[48];

            // Accumulate and clamp.
            jOld1 = data[41];
            var jOld2 = data[42];
            cjAcc1 = jOld1 + j1;
            var cjAcc2 = jOld2 + j2;

            var jMax = friction * data[40];
            var fsq = (cjAcc1 * cjAcc1) + (cjAcc2 * cjAcc2);
            if (fsq > (jMax * jMax))
            {
                fsq = jMax / Math.sqrt(fsq);
                cjAcc1 *= fsq;
                cjAcc2 *= fsq;
                j1 = cjAcc1 - jOld1;
                j2 = cjAcc2 - jOld2;
            }
            data[41] = cjAcc1;
            data[42] = cjAcc2;

            // Apply friction impulse.
            n0 = (u0 * j1) + (v0 * j2);
            n1 = (u1 * j1) + (v1 * j2);
            n2 = (u2 * j1) + (v2 * j2);

            va0 += (n0 * imA);
            va1 += (n1 * imA);
            va2 += (n2 * imA);

            vb0 -= (n0 * imB);
            vb1 -= (n1 * imB);
            vb2 -= (n2 * imB);

            wa0 += (data[28] * j1) + (data[34] * j2);
            wa1 += (data[29] * j1) + (data[35] * j2);
            wa2 += (data[30] * j1) + (data[36] * j2);

            wb0 += (data[31] * j1) + (data[37] * j2);
            wb1 += (data[32] * j1) + (data[38] * j2);
            wb2 += (data[33] * j1) + (data[39] * j2);
        }

        // Set local vars to velocities.
        vec = objectA.velocity;
        vec[0] = va0;
        vec[1] = va1;
        vec[2] = va2;
        vec[3] = wa0;
        vec[4] = wa1;
        vec[5] = wa2;

        vec = objectB.velocity;
        vec[0] = vb0;
        vec[1] = vb1;
        vec[2] = vb2;
        vec[3] = wb0;
        vec[4] = wb1;
        vec[5] = wb2;
    }

    invalidateParameters() {
        this.restitution = (this.objectA.restitution * this.objectB.restitution);
        this.friction = (this.objectA.friction * this.objectB.friction);
    }

    //
    // Arbiter objects are object pooled due to being frequently
    // created and destroyed.
    //
    // Arbiters are thus instead allocated an deallocated with no
    // create method.
    //
    // Object pool for arbiters

    static final List<WebGLPhysicsArbiter> arbiterPool = [];

    static WebGLPhysicsArbiter allocate(WebGLPhysicsShape shapeA,
                                        WebGLPhysicsShape shapeB,
                                        WebGLPhysicsPrivateBody objectA,
                                        WebGLPhysicsPrivateBody objectB) {
        var arbiter;
        if (arbiterPool.isEmpty) {
            arbiter = new WebGLPhysicsArbiter();
        } else {
            arbiter = arbiterPool.removeLast();
        }

        arbiter.active = true;

        arbiter.shapeA = shapeA;
        arbiter.shapeB = shapeB;
        arbiter.objectA = objectA;
        arbiter.objectB = objectB;
        arbiter.invalidateParameters();

        return arbiter;
    }

    static void deallocate(WebGLPhysicsArbiter arbiter) {
        // Prevent object pooled arbiter from keeping shapes/objects
        // from potential GC.
        arbiter.shapeA = null;
        arbiter.shapeB = null;
        arbiter.objectA = null;
        arbiter.objectB = null;

        // Ensure flag is reset.
        arbiter.skipDiscreteCollisions = false;

        // clear contact information
        arbiter.activeContacts.length = 0;
        arbiter.contactFlags = 0;
        arbiter.trigger = false;
        arbiterPool.add(arbiter);
    }
}
