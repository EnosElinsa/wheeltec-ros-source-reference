#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace wheeltec {
struct CommandLease {
  double timeout{.25}, last{-1}, vx{0}, wz{0};
  void Set(double v,double w,double now) {vx=v;wz=w;last=now;}
  std::pair<double,double> Resolve(double now,bool paused=false) const {
    if (paused || last<0 || now<last || now-last>timeout) return {0.,0.};
    return {vx,wz};
  }
};

// Rear axle reference, array: rear L/R, front L/R wheel rad/s, steer L/R rad.
inline std::array<double,6> Ackermann(double v,double w,double length,double track,
                                    double radius,double limit,double minRadius) {
  v=std::clamp(v,-.3,.3);
  w=std::abs(v)>1e-5 ? std::clamp(w,-std::abs(v)/minRadius,std::abs(v)/minRadius) : 0;
  const double k=std::abs(v)>1e-5 ? w/v : 0;
  std::array<double,6> result{};
  for (size_t side=0;side<2;++side) {
    const double y=side==0 ? track/2 : -track/2;
    result[side]=(v-y*w)/radius;
    result[side+2]=std::copysign(std::hypot(v-y*w,length*w),v)/radius;
    result[side+4]=std::clamp(std::atan2(length*k,1-y*k),-limit,limit);
  }
  return result;
}
}
