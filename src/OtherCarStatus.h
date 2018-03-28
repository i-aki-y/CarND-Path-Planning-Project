//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_SITUATIONINFO_H
#define PATH_PLANNING_SITUATIONINFO_H

#include "SensorFusion.h"

class OtherCarStatus {
 public:
  SensorFusion *forward_car_ = nullptr;
  SensorFusion *backward_car_ = nullptr;
  SensorFusion *left_forward_car_ = nullptr;
  SensorFusion *right_forward_car_;
  SensorFusion *left_backward_car_;
  SensorFusion *right_backward_car_;

  bool too_close_ = false;
  bool can_left_ = false;
  bool can_right_ = false;

};

#endif //PATH_PLANNING_SITUATIONINFO_H
