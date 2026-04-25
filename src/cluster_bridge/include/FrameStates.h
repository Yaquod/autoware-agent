//
// Created by wafdy on 2/26/26.
//

#ifndef VEHICLEAUTOWAREAGENT_FRAMESTATES_H
#define VEHICLEAUTOWAREAGENT_FRAMESTATES_H
#include <cstdint>

#include <vehicle_frame.grpc.pb.h>

struct FrameState {
  float speed_mps{0.0}, speed_kmh{0.0};
  vehicle_frame::GearState gear{vehicle_frame::GEAR_UNKNOWN};
  float steering_angle_deg{0.0};
  bool hazard_on{false};
  vehicle_frame::TurnSignal turn_signal{vehicle_frame::TURN_NONE};
  vehicle_frame::ControlMode control_mode{vehicle_frame::MODE_UNKNOWN};
  float battery_pct{0.0};
  float accel_mps2{0.0};
  float yaw_rate{0.0};
  vehicle_frame::MotionState motion_state{vehicle_frame::MOTION_UNKNOWN};
  float target_speed_mps{0.0};
  float speed_limit_mps{0.0};
  float remaining_distance_m{0.0};
  float remaining_time_s{0.0};
  bool traffic_light_red{false};
  bool traffic_light_yellow{false};
  bool traffic_light_green{false};
  uint32_t obstacle_count{0};
  uint32_t pedestrian_count{0};
  uint32_t vehicle_count{0};
  std::vector<vehicle_frame::SurroundObject> surround_objects{};
  vehicle_frame::MrmBehavior mrm_behavior{vehicle_frame::MRM_NONE};
  bool is_active{false};
  std::string description{""};
  double longitude{0.0};
  double latitude{0.0};
};
#endif  // VEHICLEAUTOWAREAGENT_FRAMESTATES_H
