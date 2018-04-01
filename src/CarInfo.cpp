//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#include <math.h>
#include "CarInfo.h"


CarInfo::CarInfo(double x, double y, double s, double d, double yaw, double speed) {

  x_ = x;
  y_ = y;
  s_ = s;
  d_ = d;
  yaw_ = yaw;
  speed_ = speed;
}

int CarInfo::GetLaneNum(double lane_width){
  return (int) (d_ / lane_width);
}

// check d is in the n_th lane;
bool CarInfo::IsSameLaneOf(CarInfo car, double lane_width = 4) {
  return GetLaneNum(lane_width) == car.GetLaneNum(lane_width);
}

bool CarInfo::IsLeftLaneOf(CarInfo car, double lane_width = 4) {
  return GetLaneNum(lane_width) + 1 == car.GetLaneNum(lane_width);
}

bool CarInfo::IsRightLaneOf(CarInfo car, double lane_width = 4) {
  return GetLaneNum(lane_width) - 1 == car.GetLaneNum(lane_width);
}

CarInfo CarInfo::CreateFromVelocity(double x, double y, double s, double d, double vx, double vy) {
  return CarInfo(x, y, s, d, atan2(vy, vx), sqrt(vx*vx + vy*vy));
}

double CarInfo::GetPrevX() {
  return x_ - cos(yaw_);
}

double CarInfo::GetPrevY() {
  return y_ - sin(yaw_);
}
