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

#include "RouteConfig.h"

#include <cstdlib>
#include <filesystem>
#include <stdexcept>

#include "Config.h"

namespace fs = std::filesystem;

namespace AutowareAgent {

RouteConfig::RouteConfig(const std::string& config_file) {
  loadFromYaml(config_file);
}

const std::string& RouteConfig::getMapName() const { return map_name_; }
const MapOrigin& RouteConfig::getMapOrigin() const { return map_origin_; }
const std::vector<LaneInfo>& RouteConfig::getAllLanes() const { return lanes_; }
size_t RouteConfig::getLanesCount() const { return lanes_.size(); }

void RouteConfig::loadFromYaml(const std::string& config_file) {
  try {
    std::string full_path = resolveConfigPath(config_file);

    if (!fs::exists(full_path)) {
      throw std::runtime_error("File does not exist: " + full_path);
    }

    YAML::Node config = YAML::LoadFile(full_path);
    YAML::Node map_info = config["map_info"];
    if (!map_info) {
      throw std::runtime_error("YAML missing top-level 'map_info' key");
    }

    map_name_ = map_info["name"].as<std::string>();

    YAML::Node origin = map_info["origin"];
    if (!origin) {
      throw std::runtime_error("YAML missing 'map_info.origin' key");
    }
    map_origin_.latitude = origin["latitude"].as<double>();
    map_origin_.longitude = origin["longitude"].as<double>();
    map_origin_.local_x = origin["local_x"].as<double>();
    map_origin_.local_y = origin["local_y"].as<double>();

    if (config["fixed_start_positions"]) {
      for (const auto& item : config["fixed_start_positions"]) {
        FixedStartPosition position;
        position.name = item["name"].as<std::string>();
        position.lane_id = item["lane_id"].as<int>();

        position.gps.latitude = item["gps"]["latitude"].as<double>();
        position.gps.longitude = item["gps"]["longitude"].as<double>();

        position.local.x = item["local"]["x"].as<double>();
        position.local.y = item["local"]["y"].as<double>();
        position.local.z = item["local"]["z"].as<double>();

        position.orientation.x = item["orientation"]["x"].as<double>();
        position.orientation.y = item["orientation"]["y"].as<double>();
        position.orientation.z = item["orientation"]["z"].as<double>();
        position.orientation.w = item["orientation"]["w"].as<double>();

        default_starts_.emplace_back(position);
      }
    }

    if (!config["lanes"]) {
      throw std::runtime_error("YAML missing 'lanes' key");
    }
    for (const auto& item : config["lanes"]) {
      LaneInfo lane;
      lane.lane_id = item["lane_id"].as<int>();

      lane.gps.latitude = item["gps"]["latitude"].as<double>();
      lane.gps.longitude = item["gps"]["longitude"].as<double>();

      lane.local.x = item["local"]["x"].as<double>();
      lane.local.y = item["local"]["y"].as<double>();
      lane.local.z = item["local"]["z"].as<double>();

      lane.orientation.z = item["orientation"]["z"].as<double>();
      lane.orientation.w = item["orientation"]["w"].as<double>();
      lane.orientation.yaw_degrees =
          item["orientation"]["yaw_degrees"].as<double>();

      lanes_.emplace_back(lane);
    }

  } catch (const YAML::BadFile& e) {
    throw std::runtime_error("failed to load route config: " +
                             std::string(e.what()));
  } catch (const std::exception& e) {
    throw std::runtime_error("failed to load route config: " +
                             std::string(e.what()));
  }
}

LocalCoordinate RouteConfig::gpsToLocalCoordinate(
    const GPSCoordinate& gps) const {
  double dlat = (gps.latitude - map_origin_.latitude) * M_PI / 180.0;
  double dlon = (gps.longitude - map_origin_.longitude) * M_PI / 180.0;
  double lat_rad = map_origin_.latitude * M_PI / 180.0;

  double dx = dlon * EARTH_RADIUS * std::cos(lat_rad);
  double dy = dlat * EARTH_RADIUS;

  return {map_origin_.local_x + dx, map_origin_.local_y + dy, 0.0};
}

const LaneInfo* RouteConfig::FindNearestLane(const GPSCoordinate& gps) const {
  LocalCoordinate target = gpsToLocalCoordinate(gps);

  double min_dist = std::numeric_limits<double>::max();
  const LaneInfo* nearest = nullptr;

  for (const auto& lane : lanes_) {
    double dx = lane.local.x - target.x;
    double dy = lane.local.y - target.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      nearest = &lane;
    }
  }
  return nearest;
}

const LaneInfo* RouteConfig::getLaneByID(int lane_id) const {
  for (const auto& lane : lanes_) {
    if (lane.lane_id == lane_id) return &lane;
  }
  return nullptr;
}

const FixedStartPosition* RouteConfig::getDefaultStart() const {
  return default_starts_.empty() ? nullptr : &default_starts_[0];
}

std::string RouteConfig::resolveConfigPath(const std::string& filename) {
  fs::path input(filename);
  if (input.is_absolute()) return input.string();

  fs::path test_path = fs::path(AutowareAgent::TEST_MAP_DIR) / filename;
  if (fs::exists(test_path)) return test_path.string();

  fs::path src_path = fs::path(AutowareAgent::SRC_MAP_DIR) / filename;
  if (fs::exists(src_path)) return src_path.string();

  fs::path install_path = fs::path(AutowareAgent::INSTALL_MAP_DIR) / filename;
  if (fs::exists(install_path)) return install_path.string();

  throw std::runtime_error("Could not find config file: " + filename);
}

}  // namespace AutowareAgent