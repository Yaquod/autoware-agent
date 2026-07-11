//
// Created by alaa on 9/7/26.
//

#ifndef VEHICLEAUTOWAREAGENT_LOCATION_FRAMESTATES_H
#define VEHICLEAUTOWAREAGENT_LOCATION_FRAMESTATES_H
#include <cstdint>
#include <vector>

#include <vehicle_frame.grpc.pb.h>

struct LocationFrameState {

  vehicle_frame::VehicleLocation vehicle_location{};
};
#endif
