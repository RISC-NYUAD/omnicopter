#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <Eigen/Core>
#include <Eigen/Sparse>


// ========================== Quaternion utils ==========================
struct Quat { double x{0}, y{0}, z{0}, w{1}; };

static inline void q_normalize(Quat& q) {
  const double n = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
  if (n > 1e-12) { q.x/=n; q.y/=n; q.z/=n; q.w/=n; } else { q = {0,0,0,1}; }
}
static inline Quat q_from_array(const double* p) { Quat q{p[0],p[1],p[2],p[3]}; q_normalize(q); return q; }
static inline void q_to_array(const Quat& q, double* p) { p[0]=q.x; p[1]=q.y; p[2]=q.z; p[3]=q.w; }
static inline double q_dot(const Quat& a, const Quat& b){ return a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w; }
static inline Quat q_neg(const Quat& q){ return Quat{-q.x,-q.y,-q.z,-q.w}; }
static inline double q_angle(const Quat& a_, const Quat& b_) {
  Quat a=a_, b=b_;
  double d = q_dot(a,b);
  if (d < 0.0) { b = q_neg(b); d = -d; }
  d = std::min(1.0, std::max(-1.0, d));
  return 2.0 * std::acos(d); // [0,pi]
}
static inline Quat q_slerp(Quat a, Quat b, double t) {
  double d = q_dot(a,b);
  if (d < 0.0) { b = q_neg(b); d = -d; }
  d = std::min(1.0, std::max(-1.0, d));
  const double EPS = 1e-8;
  if (1.0 - d < EPS) {
    Quat r{ (1-t)*a.x + t*b.x, (1-t)*a.y + t*b.y, (1-t)*a.z + t*b.z, (1-t)*a.w + t*b.w };
    q_normalize(r); return r;
  }
  double theta = std::acos(d);
  double s = std::sin(theta);
  double w1 = std::sin((1.0 - t)*theta) / s;
  double w2 = std::sin(t*theta) / s;
  Quat r{ w1*a.x + w2*b.x, w1*a.y + w2*b.y, w1*a.z + w2*b.z, w1*a.w + w2*b.w };
  q_normalize(r);
  return r;
}

static inline std::array<double,3> rotate_by_quat(const Quat& q, const std::array<double,3>& v)
{
  const double xx = q.x*q.x, yy = q.y*q.y, zz = q.z*q.z;
  const double xy = q.x*q.y, xz = q.x*q.z, yz = q.y*q.z;
  const double wx = q.w*q.x, wy = q.w*q.y, wz = q.w*q.z;

  const double r00 = 1.0 - 2.0*(yy + zz);
  const double r01 = 2.0*(xy - wz);
  const double r02 = 2.0*(xz + wy);

  const double r10 = 2.0*(xy + wz);
  const double r11 = 1.0 - 2.0*(xx + zz);
  const double r12 = 2.0*(yz - wx);

  const double r20 = 2.0*(xz - wy);
  const double r21 = 2.0*(yz + wx);
  const double r22 = 1.0 - 2.0*(xx + yy);

  return {
    r00*v[0] + r01*v[1] + r02*v[2],
    r10*v[0] + r11*v[1] + r12*v[2],
    r20*v[0] + r21*v[1] + r22*v[2]
  };
}

static Quat q_mul(const Quat& a, const Quat& b) {
  Quat q;
  q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
  q.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
  q.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
  q.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
  q_normalize(q);
  return q;
}


static Quat quat_from_rpy(double roll, double pitch, double yaw) {
  double cr = cos(roll  * 0.5);
  double sr = sin(roll  * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cy = cos(yaw   * 0.5);
  double sy = sin(yaw   * 0.5);

  Quat q;
  q.w = cr*cp*cy + sr*sp*sy;
  q.x = sr*cp*cy - cr*sp*sy;
  q.y = cr*sp*cy + sr*cp*sy;
  q.z = cr*cp*sy - sr*sp*cy;
  q_normalize(q);
  return q;
}


static void dq_dRPY(double r, double p, double y,
                           Quat& dq_dr, Quat& dq_dp, Quat& dq_dy)
{
  // r=roll (X), p=pitch (Y), y=yaw (Z)
  const double cr = std::cos(0.5*r), sr = std::sin(0.5*r);
  const double cp = std::cos(0.5*p), sp = std::sin(0.5*p);
  const double cy = std::cos(0.5*y), sy = std::sin(0.5*y);

  // Base quaternion:
  // w = cr*cp*cy + sr*sp*sy
  // x = sr*cp*cy - cr*sp*sy
  // y = cr*sp*cy + sr*cp*sy
  // z = cr*cp*sy - sr*sp*cy

  // ∂q/∂roll
  dq_dr.w = -0.5*sr*cp*cy + 0.5*cr*sp*sy;
  dq_dr.x =  0.5*cr*cp*cy + 0.5*sr*sp*sy;
  dq_dr.y = -0.5*sr*sp*cy + 0.5*cr*cp*sy;
  dq_dr.z = -0.5*sr*cp*sy - 0.5*cr*sp*cy;

  // ∂q/∂pitch
  dq_dp.w = -0.5*cr*sp*cy + 0.5*sr*cp*sy;
  dq_dp.x = -0.5*sr*sp*cy - 0.5*cr*cp*sy;
  dq_dp.y =  0.5*cr*cp*cy + 0.5*sr*sp*sy;
  dq_dp.z = -0.5*cr*sp*sy + 0.5*sr*cp*cy;

  // ∂q/∂yaw
  dq_dy.w = -0.5*cr*cp*sy + 0.5*sr*sp*cy;
  dq_dy.x = -0.5*sr*cp*sy - 0.5*cr*sp*cy;
  dq_dy.y = -0.5*cr*sp*sy + 0.5*sr*cp*cy;
  dq_dy.z =  0.5*cr*cp*cy + 0.5*sr*cp*sy;
}
