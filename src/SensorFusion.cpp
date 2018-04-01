//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#include "SensorFusion.h"

SensorFusion::SensorFusion(int id, double x, double y, double vx, double vy, double s, double d) {

  id_ = id;
  car_ = CarInfo::CreateFromVelocity(x, y, s, d, vx, vy);
}

SensorFusion::~SensorFusion() = default;
