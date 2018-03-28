//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <math.h>
#include <vector>

namespace Utils {

class Segment {
 public:
  Segment(double, double, double, double);
  double x_;
  double y_;
  double x_prev_;
  double y_prev_;
  double GetYaw();
};

static std::vector<double> getXY(double s,
                                 double d,
                                 const std::vector<double> &maps_s,
                                 const std::vector<double> &maps_x,
                                 const std::vector<double> &maps_y);

static inline std::vector<double> TransformCoordinate(double x, double y, double ref_x, double ref_y, double ref_yaw) {

  double shift_x = x - ref_x;
  double shift_y = y - ref_y;

  x = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
  y = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

  return {x, y};
}

};


inline std::vector<double> TransformBackCoordinate(double x, double y, double ref_x, double ref_y, double ref_yaw) {

  double x_back = (x * cos(ref_yaw) - y * sin(ref_yaw));
  double y_back = (x * sin(ref_yaw) + y * cos(ref_yaw));

  x_back += ref_x;
  y_back += ref_y;

  return {x_back, y_back};
}




#endif //PATH_PLANNING_UTILS_H
