part of dullet_physics;



class MathHelper {
  static double min(double a, double b, double c) {

  }
  static double max(double a, double b, double c) {

  }

  static const double precision = 1e-6;
  static const double FLOAT_MAX = 3.402823466e+38;


  static Float32List m43BuildIdentity([dst]) {
    var res = dst;
    if (res == null) {
      res = new Float32List(12);
    }
    res[0] = 1.0;
    res[1] = 0.0;
    res[2] = 0.0;
    res[3] = 0.0;
    res[4] = 1.0;
    res[5] = 0.0;
    res[6] = 0.0;
    res[7] = 0.0;
    res[8] = 1.0;
    res[9] = 0.0;
    res[10] = 0.0;
    res[11] = 0.0;

    return res;
  }
  static Float32List m43Copy(Float32List m, [Float32List dst]) {
    var res = dst;
    if (res == null) {
      res = new Float32List(12);
    }
    res[0] = m[0];
    res[1] = m[1];
    res[2] = m[2];
    res[3] = m[3];
    res[4] = m[4];
    res[5] = m[5];
    res[6] = m[6];
    res[7] = m[7];
    res[8] = m[8];
    res[9] = m[9];
    res[10] = m[10];
    res[11] = m[11];

    return res;
  }
  static Float32List m43InverseOrthonormalTransformVector(Float32List m, Float32List v, [Float32List dst]) {
    if (dst == null) {
      dst = new Float32List(3);
    }
    var v0 = v[0];
    var v1 = v[1];
    var v2 = v[2];
    dst[0] = (m[0] * v0 + m[1] * v1 + m[2] * v2);
    dst[1] = (m[3] * v0 + m[4] * v1 + m[5] * v2);
    dst[2] = (m[6] * v0 + m[7] * v1 + m[8] * v2);
    return dst;
  }

  static Float32List m43InverseOrthonormalTransformPoint(Float32List m, Float32List v, [Float32List dst]) {
    if (dst == null) {
      dst = new Float32List(3);
    }
    var v0 = v[0] - m[9];
    var v1 = v[1] - m[10];
    var v2 = v[2] - m[11];
    dst[0] = (m[0] * v0 + m[1] * v1 + m[2] * v2);
    dst[1] = (m[3] * v0 + m[4] * v1 + m[5] * v2);
    dst[2] = (m[6] * v0 + m[7] * v1 + m[8] * v2);
    return dst;
  }
  static Float32List m43TransformVector(Float32List m, Float32List v, [Float32List dst]){
    var res = dst;
    if (res == null) {
      res = new Float32List(3);
    }

    var v0 = v[0];
    var v1 = v[1];
    var v2 = v[2];
    res[0] = (m[0] * v0 + m[3] * v1 + m[6] * v2);
    res[1] = (m[1] * v0 + m[4] * v1 + m[7] * v2);
    res[2] = (m[2] * v0 + m[5] * v1 + m[8] * v2);
    return res;
  }

  static Float32List m43TransformPoint(Float32List m, Float32List v, [Float32List dst]) {
    var res = dst;
    if (res == null) {
      res = new Float32List(3);
    }
    var v0 = v[0];
    var v1 = v[1];
    var v2 = v[2];
    res[0] = (m[0] * v0 + m[3] * v1 + m[6] * v2 + m[9]);
    res[1] = (m[1] * v0 + m[4] * v1 + m[7] * v2 + m[10]);
    res[2] = (m[2] * v0 + m[5] * v1 + m[8] * v2 + m[11]);
    return res;
  }
}