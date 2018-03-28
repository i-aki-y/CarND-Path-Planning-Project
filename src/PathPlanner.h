//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "Utils.h"
#include "CarInfo.h"
#include "SensorFusion.h"

class PathPlanner {
 public:
  PathPlanner(double init_ref_velocity, int init_lane_num,
              double forward_distance_threshold,
              double delta_t, double lane_width);

  const double delta_t_;
  const double lane_width_;
  const double forward_distance_threshold_;

  int target_lane_num_;
  double ref_velocity_;

  bool too_close_ = false;

  CarInfo target_car_ = CarInfo(0, 0, 0, 0, 0, 0);


  bool WillCloseTo(CarInfo other_car);

  void ResetCurrentPathInfo(CarInfo current_car,
                            std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                            double end_path_s, double end_path_d);

  void UpdateTarget();
  void UpdateOtherCarStatus(std::vector<SensorFusion> sensor_fusion_vec);
  void CreateControlPoints(std::vector<double> &ptsx, std::vector<double> &ptsy,
                           const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

  Utils::Segment GetLastSegment();

  size_t prev_size_;
  std::vector<double> previous_path_x_;
  std::vector<double> previous_path_y_;
  double end_path_s_;
  double end_path_d_;


 private:

  void UpdateTargetLaneNum();
  void UpdateRefVelocity();

};

#endif //PATH_PLANNING_PATHPLANNER_H
