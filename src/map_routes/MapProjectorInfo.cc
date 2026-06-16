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

#include "MapProjectorInfo.h"

#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace autoware_agent {

MapProjectorInfo MapProjectorInfo::load(const std::string& map_path) {
  MapProjectorInfo info;
  info.projector_type = "MGRS";

  try {
    YAML::Node cfg = YAML::LoadFile(map_path + "/map_config.yaml");

    if (cfg["projector_type"]) {
      info.projector_type = cfg["projector_type"].as<std::string>("MGRS");
    }

    YAML::Node origin = cfg["map_config"];
    if (!origin) {
      throw std::runtime_error("map_config.yaml: missing 'map_config' section");
    }
    info.origin_lat = origin["latitude"].as<double>(0.0);
    info.origin_lon = origin["longitude"].as<double>(0.0);
    info.elevation = origin["elevation"].as<double>(0.0);
    


    if (cfg["local_offset"]) {
      info.local_offset_x = cfg["local_offset"]["x"].as<double>(0.0);
      info.local_offset_y = cfg["local_offset"]["y"].as<double>(0.0);
    }

    if (cfg["start_position"]) {
      YAML::Node sp = cfg["start_position"];
      info.has_start = true;
      info.start_name = sp["name"].as<std::string>("default");
      info.start_lat = sp["latitude"].as<double>(info.origin_lat);
      info.start_lon = sp["longitude"].as<double>(info.origin_lon);
    }

  } catch (const YAML::BadFile& e) {
    throw std::runtime_error("Failed to load map_config.yaml: " + std::string(e.what()));
  }

  return info;
}

}  // namespace autoware_agent