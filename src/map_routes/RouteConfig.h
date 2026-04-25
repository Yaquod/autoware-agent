/*
 * Copyright 2026 wafdy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef AUTOWARE_CARLA_GNSS_ROUTECONFIG_H
#define AUTOWARE_CARLA_GNSS_ROUTECONFIG_H

#include <cmath>
#include <string>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>
#include <yaml-cpp/yaml.h>

namespace autoware_agent {
struct GPSCoordinate {
  double latitude;
  double longitude;
};

struct LocalCoordinate {
  double x;
  double y;
  double z;
};

struct Orientation {
  double x = 0.0;
  double y = 0.0;
  double z;
  double w;
  double yaw_degrees;
};

struct LaneInfo {
  int lane_id;
  GPSCoordinate gps;
  LocalCoordinate local;
  Orientation orientation;
};

struct MapOrigin {
  double latitude;
  double longitude;
  double local_x;
  double local_y;
};

struct FixedStartPosition {
  std::string name;
  int lane_id;
  GPSCoordinate gps;
  LocalCoordinate local;
  Orientation orientation;
};

class RouteConfig {
 public:
  RouteConfig(const std::string& config_file);

  ~RouteConfig() = default;

  const std::string& getMapName() const;

  const MapOrigin& getMapOrigin() const;

  const std::vector<LaneInfo>& getAllLanes() const;

  [[nodiscard]] size_t getLanesCount() const;

  void loadFromYaml(const std::string& config_file);

  [[nodiscard]] LocalCoordinate gpsToLocalCoordinate(const GPSCoordinate& gps) const;

  [[nodiscard]] GPSCoordinate localCoordinateToGps(const LocalCoordinate& local) const;

  [[nodiscard]] const LaneInfo* findNearestLane(const GPSCoordinate& gps) const;

  [[nodiscard]] const LaneInfo* getLaneByID(int lane_id) const;

  [[nodiscard]] const FixedStartPosition* getDefaultStart() const;

 private:
  std::string map_name_;
  MapOrigin map_origin_{};
  std::vector<LaneInfo> lanes_;
  std::vector<FixedStartPosition> default_starts_;
  static constexpr double EARTH_RADIUS = 6378137.0;

  static std::string resolveConfigPath(const std::string& filename);
};
}  // namespace autoware_agent

#endif  // AUTOWARE_CARLA_GNSS_ROUTECONFIG_H
