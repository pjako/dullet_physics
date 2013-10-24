part of dullet_physics;




//
// WebGLPhysicsContact
//

// [0,  3) : localA (vector3)
// [3,  6) : localB (vector3)
// [6,  9) : relA   (vector3)
// [9, 12) : relB   (vector3)
// [12,15) : normal (vector3)  ('on' object B)
// [15,18) : tangent (vector3)
// [18,21) : bitangent (vector3)
// [21,22) : distance
// [22,25) : Jacobian : normal x relA * iInertiaA (vector3)
// [25,28) : Jacobian : normal x relB * iInertiaB (vector3)
// [28,31) : Jacobian : tangent x relA * iInertiaA (vector3)
// [31,34) : Jacobian : tangent x relB * iInertiaB (vector3)
// [34,37) : Jacobian : bitangent x relA * iInertiaA (vector3)
// [37,40) : Jacobian : bitangent x relB * iInertiaB (vector3)
// [40,43) : jAcc (normal, tangent, bitangent) (vector3)
// [43,44) : bias
// [44,45) : jAccBias
// [45,46) : penetration constraint, effective mass (kN)
// [46,50) : friction constraint, effective mass
//           [ kfA   kfBC ]
//           [ kfBC   kfD ]
// [50,51) : bounce
// [51,52) : new
//

//
// Contact objects are object pooled due to being frequently
// created and destroyed.
//
// Contacts are thus instead allocated an deallocated with no
// create method.
//
class  WebGLPhysicsContact /*extends Float32Array*/ {
  final Float32List storage = new Float32List(12);

  double operator[](int i) => storage[i];

  void operator[]=(int i, double v) { storage[i] = v; }

  static final List<WebGLPhysicsContact> contactPool = <WebGLPhysicsContact>[];//: <WebGLPhysicsContact[]>[],

  static final List<WebGLPhysicsPublicContact> publicContacts = [new WebGLPhysicsPublicContact(), new WebGLPhysicsPublicContact(), new WebGLPhysicsPublicContact()];
  static final List<WebGLPhysicsPublicContact> callbackContacts = <WebGLPhysicsPublicContact>[];//: <WebGLPhysicsPublicContact[]>[],

  static WebGLPhysicsContact allocate() {
      WebGLPhysicsContact contact;
      if (contactPool.isEmpty)
      {
          contact = new WebGLPhysicsContact();//new Float32List(52);
      } else {
          contact = contactPool.removeLast();
      }

      contact[51] = 1.0; // new contact

      return contact;
  }

  static void deallocate(WebGLPhysicsContact contact){
      contactPool.add(contact);
      // Contact jAccN is cached between updates. Needs to be reset if contact is re-used.
      contact[40] = 0;
  }
}
