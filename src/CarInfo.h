//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_CARINFO_H
#define PATH_PLANNING_CARINFO_H

class CarInfo {
 public:

  CarInfo(double x, double y, double s, double d, double yaw, double speed);

  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  double speed_;

  static CarInfo CreateFromVelocity(double x, double y, double s, double d, double vx, double vy);

  int GetLaneNum(double lane_width);
  bool IsSameLaneOf(CarInfo car, double lane_width);
  bool IsLeftLaneOf(CarInfo car, double lane_width);
  bool IsRightLaneOf(CarInfo car, double lane_width);
  double GetPrevX();
  double GetPrevY();
};


#endif //PATH_PLANNING_CARINFO_H
