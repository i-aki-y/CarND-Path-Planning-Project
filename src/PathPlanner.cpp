//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#include <math.h>
#include "PathPlanner.h"

using namespace std;

PathPlanner::PathPlanner(double init_ref_velocity, int init_lane_num,
                         double forward_distance_threshold,
                         double delta_t, double lane_width):
    ref_velocity_(init_ref_velocity),
    target_lane_num_(init_lane_num),
    forward_distance_threshold_(forward_distance_threshold),
    delta_t_(delta_t),
    lane_width_(lane_width)
{

}


void PathPlanner::UpdateTarget() {
  UpdateTargetLaneNum();
  UpdateRefVelocity();
}

void PathPlanner::UpdateOtherCarStatus(std::vector<SensorFusion> sensor_fusion_vec){
  // find ref_v to use
  for (SensorFusion sf : sensor_fusion_vec) {
    // car is in my lane
    if (sf.car_info_.IsLane(target_lane_num_, lane_width_)) {
      double check_car_s = sf.car_info_.s_ + (static_cast<double>(prev_size_) * delta_t_ * sf.car_info_.speed_);

      if ((check_car_s > target_car_.s_) && ((check_car_s - target_car_.s_) < forward_distance_threshold_)) {
        // ref_vel = 29.5; //mph
        // cout << i << ": " << check_car_s - car_s << ": " << ref_vel << endl;
        too_close_ = true;
      }
    }
  }
}


void PathPlanner::UpdateTargetLaneNum() {
  if (target_lane_num_ == 0 && too_close_) {
    target_lane_num_ = 1;
  } else if (target_lane_num_ == 1 && too_close_) {
    target_lane_num_ = 0;
  }
}

void PathPlanner::UpdateRefVelocity() {
  if (too_close_) {
    ref_velocity_ -= 0.224;  // ~ 5 m/s^2
  } else if (ref_velocity_ < 49.5) {
    ref_velocity_ += 0.224;
  }
}
void PathPlanner::ResetCurrentPathInfo(CarInfo current_car,
                                       std::vector<double> previous_path_x,
                                       std::vector<double> previous_path_y,
                                       double end_path_s,
                                       double end_path_d) {

  target_car_ = current_car;
  previous_path_x_ = previous_path_x;
  previous_path_y_ = previous_path_y;
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;

  prev_size_ = previous_path_x_.size();

  if (prev_size_ > 0) {
    target_car_.s_ = end_path_s_;
  }


}
void PathPlanner::CreateControlPoints(std::vector<double> &ptsx, std::vector<double> &ptsy,
                                      const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y) {

  /*
  if (prev_size_ < 2) {
    ptsx.push_back(target_car_.GetPrevX());
    ptsx.push_back(target_car_.x_);

    ptsy.push_back(target_car_.GetPrevY());
    ptsy.push_back(target_car_.x_);

  } else {

    ref_x = previous_path_x_[prev_size_ - 1];
    ref_y = previous_path_y_[prev_size_ - 1];

    double ref_x_prev = previous_path_x_[prev_size_ - 2];
    double ref_y_prev = previous_path_y_[prev_size_ - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

  }

  vector<double>
      next_wp0 = Utils::getXY(target_car_.s_ + 30, lane_width_ * (target_lane_num_ + 0.5), maps_s, maps_x, maps_y);
  vector<double>
      next_wp1 = Utils::getXY(target_car_.s_ + 60, lane_width_ * (target_lane_num_ + 0.5), maps_s, maps_x, maps_y);
  vector<double>
      next_wp2 = Utils::getXY(target_car_.s_ + 90, lane_width_ * (target_lane_num_ + 0.5), maps_s, maps_x, maps_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {

    // transform coordinate
    auto xy = Utils::TransformCoordinate(ptsx[i], ptsy[i], ref_x, ref_y, ref_yaw);
    ptsx[i] = xy[0];
    ptsy[i] = xy[1];
  }
  */
}


Utils::Segment PathPlanner::GetLastSegment() {

  if (prev_size_ < 2) {
    return Utils::Segment(target_car_.x_, target_car_.y_, target_car_.GetPrevX(), target_car_.GetPrevY());
  } else {
    double x = previous_path_x_[prev_size_ - 1];
    double y = previous_path_y_[prev_size_ - 1];

    double x_prev = previous_path_x_[prev_size_ - 2];
    double y_prev = previous_path_y_[prev_size_ - 2];

    return Utils::Segment(x, y, x_prev, y_prev);
  }
}
