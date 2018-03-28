//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_LANESTATUS_H
#define PATH_PLANNING_LANESTATUS_H

#include "SensorFusion.h"

class LaneStatus {
  bool is_filled_;
  SensorFusion *forward_car_ = nullptr;
  SensorFusion *backward_car_ = nullptr;
};

#endif //PATH_PLANNING_LANESTATUS_H
