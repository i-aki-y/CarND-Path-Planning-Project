//
// Created by AkiyukiIshikawa on 2018/03/28.
//

#include <iostream>
#include <math.h>
#include <iomanip>

#include "spline.h"
#include "PathPlanner.h"

using namespace std;

PathPlanner::PathPlanner(double init_ref_velocity, int init_lane_num,
                         double forward_distance_threshold,
                         int n_sample,
                         double delta_t, double lane_width, double max_velocity, double max_s):
    ref_velocity_(init_ref_velocity),
    target_lane_num_(init_lane_num),
    forward_distance_threshold_(forward_distance_threshold),
    n_sample_(n_sample),
    delta_t_(delta_t),
    lane_width_(lane_width),
    max_velocity_(max_velocity),
    max_s_(max_s)
{

}

// Calculate next target lane number and target velocity
void PathPlanner::UpdateTarget() {
  UpdateTargetLaneNum();
  UpdateRefVelocity();
}

// Update road status from given sensor fusion data.
//
// other car is in front                      -> `detect_forward_ = true`
// other car is in front and is in very close -> `too_close_ = true`
// other car is in left lane                  -> `left_is_filled_ = true`
// other car is in right lane                 -> `right_is_filled_ = true`
//
void PathPlanner::UpdateOtherCarStatus(std::vector<SensorFusion> &sensor_fusion_vec) {

  road_status_ = RoadStatus();

  // find ref_v to use
  for (SensorFusion sf : sensor_fusion_vec) {

    double check_car_s = PredictFutureS(sf.car_);

    // car is in my lane
    if (sf.car_.IsSameLaneOf(target_car_, lane_width_)) {
      if (HasCloseForward(check_car_s)) {
        // If there is a car close
        road_status_.detect_forward_ = true;
        if (HasTooCloseForward(sf.car_.s_)) {
          road_status_.too_close_ = true;
        }

        if (road_status_.forward_car_ != nullptr) {
          double new_ds = Utils::NormalizedDiff(target_car_.s_, check_car_s, max_s_);

          // get current forward car's future distance in s
          double cur_s = PredictFutureS(*road_status_.forward_car_);
          double cur_ds = Utils::NormalizedDiff(target_car_.s_, cur_s, max_s_);

          // In order to adjust the velocity later, the nearest car in front is stored
          if (new_ds < cur_ds) {
            // if new distance of s closer than current s, update forward_car
            (*road_status_.forward_car_) = sf.car_;
          }
        } else {
          road_status_.forward_car_ = new CarInfo(sf.car_);
        }
      }
    } else if (sf.car_.IsLeftLaneOf(target_car_, lane_width_)) {
      if (IsFilled(check_car_s)) {
        road_status_.left_is_filled_ = true;
      }
    } else if (sf.car_.IsRightLaneOf(target_car_, lane_width_)) {
      if (IsFilled(check_car_s)) {
        road_status_.right_is_filled_ = true;
      }
    }
  }
}

// Update target lane
// When current lane != target lane
void PathPlanner::UpdateTargetLaneNum() {
  // current lane is detected from the current car position.
  int cur_lane_num = target_car_.GetLaneNum(lane_width_);

  // If cuurent lane and target lane is not same,
  // it is considered that the car is switching the lane.
  if (cur_lane_num != target_lane_num_) {
    return;
  }

  // When current lane is not around the center of any lane,
  // it is considered that the lane switching operation is not completed
  if (!IsAroundLaneCenter(cur_lane_num)) {
    return;
  }

  if (road_status_.detect_forward_ && CanSwitchToLeft()) {
    // switch to left lane
    target_lane_num_--;
  } else if (road_status_.detect_forward_ && CanSwitchToRight()) {
    // switch to right lane
    target_lane_num_++;
  }

  if (!road_status_.detect_forward_) {
    if ((CanSwitchToLeft() && target_lane_num_ == 2) ||
        (CanSwitchToRight() && target_lane_num_ == 0)) {
      // now, my car is the left or right lane and the middle lane is empty
      target_lane_num_ = 1;
    }
  }
}

void PathPlanner::UpdateRefVelocity() {

  if (road_status_.detect_forward_) {
    // Detected a forward car which is close to the target car.

    CarInfo fw_car = *road_status_.forward_car_;
    double fw_car_mph = Utils::ToMPH(fw_car.speed_);
    double fw_car_ds = Utils::NormalizedDiff(target_car_.s_, fw_car.s_, max_s_);

    /*
    cout << std::fixed << std::setprecision(2)
         << ref_velocity_
         << " : " << fw_car_mph
         << " : " << fw_car_ds
         << " : " << target_car_.s_
         << " : " << fw_car.s_
         << endl;
    */

    if (CanSwitchToLeft() || CanSwitchToRight()) {
      // If my car can switch lane, keep the velocity.
      return;
    }

    if (road_status_.too_close_) {
      // When the forward car is too close, speed down a lot.
      //cout << "too close!" << endl;
      ref_velocity_ -= 0.268;  // ~ 6 m/s^2
    } else if (ref_velocity_ - 5 > fw_car_mph) {
      // Speed down gradually but not too much.
      ref_velocity_ -= 0.179;  //
    } else if ((ref_velocity_ < fw_car_mph) && fw_car_ds > forward_distance_threshold_) {
      // If distance between forward car and target car is larger than threshold, increase the target car speed.
      ref_velocity_ += 0.224;
    }
  } else if (ref_velocity_ < max_velocity_) {
    // If there is no forward car, increase speed until maximum speed.
    ref_velocity_ += 0.224;
  }
}

// Reset car and previous path information.
void PathPlanner::ResetCurrentPathInfo(CarInfo current_car,
                                       std::vector<double> previous_path_x,
                                       std::vector<double> previous_path_y,
                                       double end_path_s,
                                       double end_path_d) {

  target_car_ = current_car;
  previous_path_x_ = std::move(previous_path_x);
  previous_path_y_ = std::move(previous_path_y);
  end_path_s_ = end_path_s;
  end_path_d_ = end_path_d;
  prev_size_ = previous_path_x_.size();

  if (prev_size_ > 0) {
    target_car_.s_ = end_path_s_;
  }

}

// Create control points for spline curve
void PathPlanner::CreateControlPoints(const std::vector<double> &maps_s, const std::vector<double> &maps_x,
                                      const std::vector<double> &maps_y, std::vector<double> &ptsx, std::vector<double> &ptsy,
                                      double &ref_x, double &ref_y, double &ref_yaw) {


  Utils::Segment last_seg = GetLastSegment();

  ptsx.push_back(last_seg.x_prev_);
  ptsx.push_back(last_seg.x_);

  ptsy.push_back(last_seg.y_prev_);
  ptsy.push_back(last_seg.y_);

  ref_x = last_seg.x_;
  ref_y = last_seg.y_;
  ref_yaw = last_seg.GetYaw();

  // to prevent out side the lane incident.
  double modifed_lane_num = PathPlanner::GetModifedLaneNum(target_lane_num_);

  vector<double>
      next_wp0 = Utils::getXY(target_car_.s_ + 30, lane_width_ * (modifed_lane_num + 0.5), maps_s, maps_x, maps_y);
  vector<double>
      next_wp1 = Utils::getXY(target_car_.s_ + 60, lane_width_ * (modifed_lane_num + 0.5), maps_s, maps_x, maps_y);
  vector<double>
      next_wp2 = Utils::getXY(target_car_.s_ + 90, lane_width_ * (modifed_lane_num + 0.5), maps_s, maps_x, maps_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

}

//
void PathPlanner::CreateSplinePath(const std::vector<double> &ptsx,
                                   const std::vector<double> &ptsy,
                                   std::vector<double> &new_ptsx,
                                   std::vector<double> &new_ptsy) {
  tk::spline s;

  s.set_points(ptsx, ptsy);

  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

  double x_add_on = 0;

  for(int i = 1; i <= n_sample_ - prev_size_; i++) {
    double N = (target_dist / (delta_t_ * ref_velocity_ / 2.24));
    double x_point = x_add_on + (target_x) / N;
    double y_point = s(x_point);

    x_add_on = x_point;

    new_ptsx.push_back(x_point);
    new_ptsy.push_back(y_point);
  }
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
void PathPlanner::CreateNextPath(const std::vector<double> &maps_s,
                                 const std::vector<double> &maps_x,
                                 const std::vector<double> &maps_y,
                                 std::vector<double> &next_ptsx,
                                 std::vector<double> &next_ptsy) {

  vector<double> ptsx;
  vector<double> ptsy;
  vector<double> new_ptsx;
  vector<double> new_ptsy;

  double ref_x = target_car_.x_;
  double ref_y = target_car_.y_;
  double ref_yaw = target_car_.yaw_;

  CreateControlPoints(maps_s, maps_x, maps_y, ptsx, ptsy, ref_x, ref_y, ref_yaw);

  Utils::TransformCoordinate(ref_x, ref_y, ref_yaw, ptsx, ptsy);

  CreateSplinePath(ptsx, ptsy, new_ptsx, new_ptsy);

  Utils::TransformBackCoordinate(ref_x, ref_y, ref_yaw, new_ptsx, new_ptsy);

  for (int i = 0; i < prev_size_; i++) {
    next_ptsx.push_back(previous_path_x_[i]);
    next_ptsy.push_back(previous_path_y_[i]);
  }

  for (int i = 0; i < new_ptsx.size(); i++) {
    next_ptsx.push_back(new_ptsx[i]);
    next_ptsy.push_back(new_ptsy[i]);
  }

}

double PathPlanner::PredictFutureS(CarInfo car) {
  return car.s_ + (static_cast<double>(prev_size_) * delta_t_ * car.speed_);
}

bool PathPlanner::HasCloseForward(double other_s){
  double ds = Utils::NormalizedDiff(target_car_.s_, other_s, max_s_);
  return (ds > 0) && (ds < forward_distance_threshold_);
}

bool PathPlanner::HasTooCloseForward(double other_s){
  double ds = Utils::NormalizedDiff(target_car_.s_, other_s, max_s_);
  return ds < 5;
}

bool PathPlanner::IsFilled(double other_s){
  double ds = Utils::NormalizedDiff(target_car_.s_, other_s, max_s_);
  return (ds + 0.5 * forward_distance_threshold_> 0) && (ds < 1.25 * forward_distance_threshold_);
}

bool PathPlanner::CanSwitchToLeft(){
  return !road_status_.left_is_filled_ && target_lane_num_ > 0;
}

bool PathPlanner::CanSwitchToRight(){
  return !road_status_.right_is_filled_ && target_lane_num_ < 2;
}

bool PathPlanner::IsAroundLaneCenter(int lane_num) {
  double center_d = (lane_num + 0.5) * lane_width_;

  return (abs(target_car_.d_ - center_d) < 0.2 * lane_width_);
}

double PathPlanner::GetModifedLaneNum(int lane_num) {
  if (lane_num == 0) {
    return 0.05;
  } else if (lane_num == 2) {
    return 1.95;
  } else {
    return 1;
  }
}