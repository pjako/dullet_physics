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
  final Float32List _storage = new Float32List(52);

  double operator[](int i) => _storage[i];

  void operator[]=(int i, double v) { _storage[i] = v; }

  static final List<WebGLPhysicsContact> _contactPool = <WebGLPhysicsContact>[];//: <WebGLPhysicsContact[]>[],

  static final List<WebGLPhysicsContact> _publicContacts = [new WebGLPhysicsContact(), new WebGLPhysicsContact(), new WebGLPhysicsContact()];
  static final List<WebGLPhysicsContact> _callbackContacts = <WebGLPhysicsContact>[];//: <WebGLPhysicsPublicContact[]>[],

  static WebGLPhysicsContact allocate() {
      WebGLPhysicsContact contact;
      if (_contactPool.isEmpty)
      {
          contact = new WebGLPhysicsContact();//new Float32List(52);
      } else {
          contact = _contactPool.removeLast();
      }

      contact._storage[51] = 1.0; // new contact

      return contact;
  }

  static void deallocate(WebGLPhysicsContact contact){
      _contactPool.add(contact);
      // Contact jAccN is cached between updates. Needs to be reset if contact is re-used.
      contact[40] = 0.0;
  }

  Vector3 get localPointOnA {
    return new Vector3(_storage[0], _storage[1], _storage[2]);
  }
  void set localPointOnA(Vector3 point) {
    _storage[0] = point.storage[0];
    _storage[1] = point.storage[1];
    _storage[2] = point.storage[2];
  }
  Vector3 get localPointOnB {
    return new Vector3(_storage[3], _storage[4], _storage[5]);
  }
  void set localPointOnB(Vector3 point) {
    _storage[3] = point.storage[0];
    _storage[4] = point.storage[1];
    _storage[5] = point.storage[2];
  }

  Vector3 get worldNormalOnB {
    return new Vector3(_storage[12], _storage[13], _storage[14]);
  }
  void set worldNormalOnB(Vector3 normal) {
    _storage[12] = normal.storage[0];
    _storage[13] = normal.storage[1];
    _storage[14] = normal.storage[2];
  }

  bool get added => (0.0 < _storage[51]);

  double get distance => _storage[21];
}


//
// WebGLPhysicsPublicContact
//
/*
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
}*/

/*class WebGLPhysicsPublicContact {
    //Float32List localPointOnA;//  : any;  // v3 - getter
    //Float32List localPointOnB;//  : any;  // v3 - getter
    //Float32List worldNormalOnB;// : any; // v3 - getter
    //bool added;//          : boolean;
    //double distance;//       : number;

    //var _private; // TODO: can this stuff just be turned into private
                   // members?




}*/
