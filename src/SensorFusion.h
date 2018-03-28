//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_SENSORFUSION_H
#define PATH_PLANNING_SENSORFUSION_H

#include "CarInfo.h"

class SensorFusion{

 public:
  SensorFusion(int id, double x, double y, double vx, double vy, double s, double d);

  virtual ~SensorFusion();

  int id_;
  CarInfo car_info_ = CarInfo(0, 0, 0, 0, 0, 0);

};


#endif //PATH_PLANNING_SENSORFUSION_H
