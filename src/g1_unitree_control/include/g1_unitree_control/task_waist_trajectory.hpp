#pragma once
#include <array>
#include <cmath>

namespace g1_task {
// A polynomial stays in the convex hull of its Bernstein coefficients. This
// conservative check covers interior Hermite extrema, not just endpoint data.
inline bool polynomial_inside(const std::array<double, 6>& power, int degree,
                              double lower, double upper)
{
  for (int k = 0; k <= degree; ++k) {
    double value = 0.;
    for (int i = 0; i <= k; ++i) {
      double ratio = 1.;
      for (int j = 0; j < i; ++j) ratio *= double(k-j)/double(degree-j);
      value += power[i]*ratio;
    }
    if (!std::isfinite(value) || value < lower-1e-6 || value > upper+1e-6) return false;
  }
  return true;
}

inline bool waist_segment_safe(double q0, double q1, double v0, double v1,
                               double a0, double a1, double dt, bool velocity,
                               bool acceleration, double lower, double upper,
                               double vmax, double amax)
{
  if (!std::isfinite(dt) || dt <= 0) return false;
  std::array<double,6> c{}; c[0]=q0;
  if (velocity) {
    c[1]=v0*dt;
    if (acceleration) {
      c[2]=a0*dt*dt/2;
      const double delta=q1-c[0]-c[1]-c[2], v=v1*dt-c[1]-2*c[2];
      const double a=a1*dt*dt-2*c[2];
      c[3]=10*delta-4*v+a/2; c[4]=-15*delta+7*v-a; c[5]=6*delta-3*v+a/2;
    } else {
      c[2]=3*(q1-q0)-2*v0*dt-v1*dt; c[3]=-2*(q1-q0)+v0*dt+v1*dt;
    }
  } else c[1]=q1-q0;
  std::array<double,6> d{}, dd{};
  for (int i=0;i<5;++i) d[i]=(i+1)*c[i+1]/dt;
  for (int i=0;i<4;++i) dd[i]=(i+1)*d[i+1]/dt;
  return polynomial_inside(c,5,lower,upper) && polynomial_inside(d,4,-vmax,vmax) &&
         polynomial_inside(dd,3,-amax,amax);
}
}  // namespace g1_task
