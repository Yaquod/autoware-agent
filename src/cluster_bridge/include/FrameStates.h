//
// Created by wafdy on 2/26/26.
//

#ifndef VEHICLEAUTOWAREAGENT_FRAMESTATES_H
#define VEHICLEAUTOWAREAGENT_FRAMESTATES_H
#include <vehicle_frame.grpc.pb.h>

struct FrameState {
  float speed_mps{0.0}, speed_kmh{0.0};
  vehicle_frame::GearState gear{vehicle_frame::GEAR_UNKNOWN};
  float steering_angle_deg{0.0};
};
#endif  // VEHICLEAUTOWAREAGENT_FRAMESTATES_H
