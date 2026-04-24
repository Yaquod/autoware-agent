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

#ifndef VEHICLE_AUTOWARE_AGENT_ZENOH_PUBLISHER_H
#define VEHICLE_AUTOWARE_AGENT_ZENOH_PUBLISHER_H

#pragma once
#include <memory>
#include <string>

#include <zenoh.hxx>

namespace autoware_agent {

class ZenohPublisher {
 public:
  explicit ZenohPublisher(const std::shared_ptr<zenoh::Session>& session, const std::string& key)
    : session_(session)
    , publisher_(session->declare_publisher(key)) {}

  void publish(const std::string& serialized_proto) {
    publisher_.put(serialized_proto);
  }
  static void shutDown() {
    /* Do Nothing */
  }

 private:
  std::shared_ptr<zenoh::Session> session_;
  zenoh::Publisher publisher_;
};
}  // namespace AutowareAgent

#endif  // VEHICLE_AUTOWARE_AGENT_ZENOH_PUBLISHER_H
