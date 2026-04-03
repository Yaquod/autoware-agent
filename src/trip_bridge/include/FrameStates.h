//
// Created by wafdy on 2/26/26.
//

#ifndef VEHICLEAUTOWAREAGENT_TRIP_FRAMESTATES_H
#define VEHICLEAUTOWAREAGENT_TRIP_FRAMESTATES_H
#include <cstdint>
#include <vector>

#include <vehicle_frame.grpc.pb.h>

struct TripFrameState {
  vehicle_frame::LocalizationState localization_state{};
  vehicle_frame::OperationMode operation_mode{};
  vehicle_frame::MrmState mrm_state{};
  bool system_alive{false};
  std::vector<vehicle_frame::ComponentHealth> components{};
  vehicle_frame::TripState trip_state{};
  std::string start_lanelet_id{};
  double start_x{0.0};
  double start_y{0.0};
  double start_z{0.0};
  std::string goal_lanelet_id{};
  double goal_x{0.0};
  double goal_y{0.0};
  double goal_z{0.0};
  double goal_distance_m{0.0};
};
#endif
