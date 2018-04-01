//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_SITUATIONINFO_H
#define PATH_PLANNING_SITUATIONINFO_H

#include "LaneStatus.h"

class RoadStatus {
 public:
  bool detect_forward_ = false;
  bool too_close_ = false;
  bool left_is_filled_ = false;
  bool right_is_filled_ = false;
  CarInfo *forward_car_ = nullptr;
};

#endif //PATH_PLANNING_SITUATIONINFO_H
