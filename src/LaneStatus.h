//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_LANESTATUS_H
#define PATH_PLANNING_LANESTATUS_H

#include "SensorFusion.h"

class LaneStatus {
 public:
  bool is_valid_ = false;
  bool is_filled_ = false;
  SensorFusion *forward_car_ = nullptr;
  SensorFusion *backward_car_ = nullptr;

  LaneStatus(bool is_valid);
  void UpdateForward(SensorFusion sf);

};

#endif //PATH_PLANNING_LANESTATUS_H
