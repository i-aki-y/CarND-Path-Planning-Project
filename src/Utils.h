//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <math.h>
#include <vector>

namespace Utils {

// This class represent a line segment
class Segment {
 public:
  Segment(double, double, double, double);
  double x_;
  double y_;
  double x_prev_;
  double y_prev_;
  double GetYaw();
};

// This function is moved from main.cpp
std::vector<double> getXY(double s,
                          double d,
                          const std::vector<double> &maps_s,
                          const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y);


// Coordinate transformation
static inline std::vector<double> TransformCoordinate(const double ref_x, const double ref_y, const double ref_yaw,
                                                      const double x, const double y) {

  double shift_x = x - ref_x;
  double shift_y = y - ref_y;

  double x_trans = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
  double y_trans = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

  return {x_trans, y_trans};
}

// Backward coordinate transformation
static inline std::vector<double> TransformBackCoordinate(const double ref_x, const double ref_y, const double ref_yaw,
                                                          const double x, const double y) {

  double x_back = (x * cos(ref_yaw) - y * sin(ref_yaw));
  double y_back = (x * sin(ref_yaw) + y * cos(ref_yaw));

  x_back += ref_x;
  y_back += ref_y;

  return {x_back, y_back};
}

// Coordinate transformation for vector variable
static inline void TransformCoordinate(const double ref_x, const double ref_y, const double ref_yaw,
                                       std::vector<double> &xs, std::vector<double> &ys) {
  size_t n = xs.size();

  for (size_t i = 0; i < n; i++) {
    std::vector<double> xy = TransformCoordinate(ref_x, ref_y, ref_yaw, xs[i], ys[i]);
    xs[i] = xy[0];
    ys[i] = xy[1];
  }
}

// Backward coordinate transformation for vector variable
static inline void TransformBackCoordinate(const double ref_x, const double ref_y, const double ref_yaw,
                                           std::vector<double> &xs, std::vector<double> &ys) {

  size_t n = xs.size();

  for (size_t i = 0; i < n; i++) {
    std::vector<double> xy = TransformBackCoordinate(ref_x, ref_y, ref_yaw, xs[i], ys[i]);
    xs[i] = xy[0];
    ys[i] = xy[1];
  }
}


// Return normalized Frenet coordinate distance: ds = (target_s - base_s).
// when target_s = 1 and base_s = 6945.554 (= max_s),
// this function returns ds = 1 not -6944.554.
static inline double NormalizedDiff(double base_s, double target_s, double max_s) {
  double ds = (target_s - base_s);
  while (ds > max_s / 2) {
    ds -= max_s;
  }
  while (ds < -max_s / 2) {
    ds += max_s;
  }
  return ds;
}

// convert velocity from m/s to mph
static inline double ToMPH(double v_ms) {
  return 2.236 * v_ms;
}

// convert velocity from mph to m/s
static inline double ToMS(double v_mps) {
  return 0.44704 * v_mps;
}

};

#endif //PATH_PLANNING_UTILS_H
