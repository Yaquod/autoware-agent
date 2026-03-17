//
// Created by wafdy on 2/26/26.
//

#ifndef VEHICLEAUTOWAREAGENT_PERCEPTION_FRAMESTATES_H
#define VEHICLEAUTOWAREAGENT_PERCEPTION_FRAMESTATES_H
#include <cstdint>
#include <vector>

#include <vehicle_frame.grpc.pb.h>

struct PerceptionFrameState {

   std::vector<vehicle_frame::SurroundingObject>surrounding_objects{};
   std::vector<vehicle_frame::PointCloud>points_cloud{};
   vehicle_frame::OccupancyGrid occupancy_grid {};
   std::vector<vehicle_frame::TrafficLight>traffic_lights {};




};
#endif
