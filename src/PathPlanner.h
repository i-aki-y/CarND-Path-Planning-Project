//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#ifndef PATH_PLANNING_PATHPLANNER_H
#define PATH_PLANNING_PATHPLANNER_H

#include <vector>
#include "Utils.h"
#include "CarInfo.h"
#include "SensorFusion.h"
#include "OtherCarStatus.h"

class PathPlanner {
 public:
  PathPlanner(double init_ref_velocity, int init_lane_num,
              double forward_distance_threshold,
              int n_sample,
              double delta_t, double lane_width, double max_velocity);

  const double delta_t_;
  const double lane_width_;
  const double forward_distance_threshold_;
  int n_sample_;
  int target_lane_num_;
  double ref_velocity_;
  double max_velocity_;

  bool too_close_ = false;

  OtherCarStatus current_situation_ = OtherCarStatus();
  CarInfo target_car_ = CarInfo(0, 0, 0, 0, 0, 0);


  bool WillCloseTo(CarInfo other_car);

  void ResetCurrentPathInfo(CarInfo current_car,
                            std::vector<double> previous_path_x, std::vector<double> previous_path_y,
                            double end_path_s, double end_path_d);

  void UpdateTarget();
  void UpdateOtherCarStatus(std::vector<SensorFusion> sensor_fusion_vec);
  void CreateControlPoints(const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y,
                           std::vector<double> &ptsx, std::vector<double> &ptsy,
                           double &ref_x, double &ref_y, double &ref_yaw);

  void CreateSplinePath(const std::vector<double> &ptsx, const std::vector<double> &ptsy,
                        const double &ref_x, const double &ref_y, const double &ref_yaw,
                        std::vector<double> &next_ptsx, std::vector<double> &next_ptsy);

  void CreateNextPath(const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y,
                      std::vector<double> &next_ptsx, std::vector<double> &next_ptsy);

  Utils::Segment GetLastSegment();

  size_t prev_size_;
  std::vector<double> previous_path_x_;
  std::vector<double> previous_path_y_;
  double end_path_s_;
  double end_path_d_;


 private:

  void UpdateTargetLaneNum();
  void UpdateRefVelocity();
  double GetFutureCarS(CarInfo car);

};

#endif //PATH_PLANNING_PATHPLANNER_H
