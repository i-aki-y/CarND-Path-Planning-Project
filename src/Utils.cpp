//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#include <iostream>
#include <math.h>
#include <vector>
#include "Utils.h"

using namespace std;

constexpr double pi() { return M_PI; }

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Utils::getXY(double s,
                            double d,
                            const vector<double> &maps_s,
                            const vector<double> &maps_x,
                            const vector<double> &maps_y) {

  int prev_wp = -1;

  while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};

}

Utils::Segment::Segment(double x, double y, double x_prev, double y_prev) {
  x_ = x;
  y_ = y;
  x_prev_ = x_prev;
  y_prev_ = y_prev;
}

double Utils::Segment::GetYaw() {
  return atan2(y_ - y_prev_, x_ - x_prev_);
}
