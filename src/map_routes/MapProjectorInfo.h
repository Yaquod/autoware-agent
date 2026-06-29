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

#ifndef VEHICLE_AUTOWARE_AGENT_MAPPROJECTORINFO_H
#define VEHICLE_AUTOWARE_AGENT_MAPPROJECTORINFO_H

#pragma once
#include <string>

namespace autoware_agent {

struct MapProjectorInfo {
  std::string projector_type;  // "MGRS" or "LocalCartesian"
  double origin_lat{};
  double origin_lon{};
  double elevation{0.0};
  double local_offset_x{0.0};
  double local_offset_y{0.0};

  bool has_start{false};
  std::string start_name;
  double start_lat{0.0};
  double start_lon{0.0};

  static MapProjectorInfo load(const std::string& map_path);
};

}  // namespace autoware_agent

#endif  // VEHICLE_AUTOWARE_AGENT_MAPPROJECTORINFO_H