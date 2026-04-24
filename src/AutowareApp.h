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
#ifndef AUTOWARE_AGENT_AUTOWARE_APP_H
#define AUTOWARE_AGENT_AUTOWARE_APP_H

#include <memory>

namespace autoware_agent {
class AutowareController;
}

class ClusterBridge;
class PlanningBridge;
class PerceptionBridge;
class TripBridge;

namespace vehicle_gateway {
class VehicleGatewayService;
}

#include <string>
#include <thread>

// forward-declare zenoh Session to avoid including zenoh headers in tests
namespace zenoh { class Session; }

namespace autoware_agent {

struct AppHandles {
  std::shared_ptr<AutowareController> controller_;
  std::shared_ptr<::ClusterBridge>      cluster_bridge_;
  std::shared_ptr<::PlanningBridge>     planning_bridge_;
  std::shared_ptr<::PerceptionBridge>   perception_bridge_;
  std::shared_ptr<::TripBridge>         trip_bridge_;
  std::shared_ptr<vehicle_gateway::VehicleGatewayService> gateway_;
  std::shared_ptr<zenoh::Session>     zsession_;
  std::thread                          ros_thread_;
};

AppHandles startAutowareApp(const std::string& yaml_path,
                           const std::string& server_addr = {},
                           const std::shared_ptr<zenoh::Session>& zsession = nullptr,
                           double controller_route_search_radius = 10.0);

void stopAutowareApp(AppHandles& handles);

} // namespace AutowareAgent

#endif // AUTOWARE_AGENT_AUTOWARE_APP_H


