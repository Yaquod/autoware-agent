//
// Created by wafdy on 2/26/26.
//

#ifndef VEHICLEAUTOWAREAGENT_PLANNING_FRAMESTATES_H
#define VEHICLEAUTOWAREAGENT_PLANNING_FRAMESTATES_H
#include <cstdint>
#include <vector>

#include <vehicle_frame.grpc.pb.h>

struct PlanningFrameState {
  std::vector<vehicle_frame::TrajectoryPoint> trajectory_points{};
  std::vector<vehicle_frame::TrajectoryPoint> lane_trajectory{};
  vehicle_frame::FullRoute full_route{};
  std::vector<vehicle_frame::VelocityFactor> velocity_factors{};
  std::vector<vehicle_frame::SteeringFactor> steering_factors{};
  float target_speed_mps{0.0f};
  float max_speed_mps{0.0f};
  vehicle_frame::EtaData eta{};
  vehicle_frame::RoutingState routing_state{};
  vehicle_frame::ScenarioState scenario_state{};
};
#endif
