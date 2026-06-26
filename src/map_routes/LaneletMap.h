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

#ifndef VEHICLE_AUTOWARE_AGENT_LANELETMAP_H
#define VEHICLE_AUTOWARE_AGENT_LANELETMAP_H

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace autoware_agent {

struct GPSCoordinate {
  double latitude{};
  double longitude{};
};

struct LocalCoordinate {
  double x{};
  double y{};
  double z{};
};

struct Orientation {
  double x{0.0};
  double y{0.0};
  double z{};
  double w{};
  double yaw_degrees{};
};

/**
 * Populated from the Lanelet2 map at query time.
 */
struct LaneInfo {
   int64_t lane_id{};
  GPSCoordinate gps{};
  LocalCoordinate local{};
  Orientation orientation{};
};

/**
 * Where the vehicle starts when Autoware boots.
 * the lanelet is resolved at runtime.
 */
struct FixedStartPosition {
  std::string name;
  GPSCoordinate gps{};
  LocalCoordinate local{};  // filled lazily after findNearestLane()
  Orientation orientation{};
};

/**
 * single source of truth for all spatial queries.
 * Loads the same lanelet2_map.osm that Autoware receives via MAP_PATH, and exposes
 * findNearestLane(GPS) for arbitrary coordinates coming from the cloud.
 */
class LaneletMap {
 public:
  LaneletMap(const std::string& osm_path, double origin_lat, double origin_lon,
             double local_offset_x = 0.0, double local_offset_y = 0.0
            
            )
             
             ;

  ~LaneletMap() = default;

  /** GPS -> Autoware local map frame (meters). */
  [[nodiscard]] LocalCoordinate gpsToLocal(const GPSCoordinate& gps) const;

  /** Autoware local map frame -> GPS. */
  [[nodiscard]] GPSCoordinate localToGps(const LocalCoordinate& local) const;
  


  //added
  void debugRouteConnectivity(int64_t from_id, int64_t to_id) const;
  void debugVerifyLocalPoint(const std::string& label,
                                        double local_x, double local_y) const;

  /**
   * Find the nearest drivable lanelet to the given GPS coordinate.
   *
   * Returns a LaneInfo whose lane_id, local, gps, and orientation are
   * derived from the Lanelet2 map centroid.  Returns nullptr when the
   * map is empty (should never happen after a successful construction).
   *
   * The returned pointer is stable for the lifetime of this LaneletMap
   * instance (results are cached internally).
   */
  [[nodiscard]] const LaneInfo* findNearestLane(const GPSCoordinate& gps) const;

  /**
   * Look up a lanelet by its integer ID.
   * Returns nullptr when the ID is not present in the map.
   */
  [[nodiscard]] const LaneInfo* getLaneById(int64_t  lane_id) const;

  /**
   * The fixed vehicle start position (e.g. depot, taxi stand).
   * Set once via setDefaultStart(); returns nullptr until then.
   */
  [[nodiscard]] const FixedStartPosition* getDefaultStart() const;

  /** Provide the starting GPS coordinate.  Resolves the nearest lanelet
   *  immediately so that getDefaultStart()->local is always populated. */
  void setDefaultStart(const std::string& name, const GPSCoordinate& gps);

  [[nodiscard]] size_t getLaneletCount() const;
  [[nodiscard]] bool isLoaded() const;


  //added
  [[nodiscard]] const LaneInfo* findNearestConnectedLane(
    const GPSCoordinate& gps, lanelet::Id reference_id, bool must_be_reachable_from_ref) const;

    [[nodiscard]] LocalCoordinate projectOntoLaneCenterline(
    const GPSCoordinate& gps, lanelet::Id lane_id) const ;

  /** Resolve an OSM file path using the same search order that
   *  RouteConfig::resolveConfigPath() used (absolute > test > src > install). */
  static std::string resolveOsmPath(const std::string& filename);

 private:
  double origin_lat_{};
  double origin_lon_{};
  double offset_x_{};
  double offset_y_{};
  double mgrs_origin_x_{0.0};
double mgrs_origin_y_{0.0};


// Add to private members of LaneletMap:
std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_;

  std::shared_ptr<lanelet::LaneletMap> map_;
  std::shared_ptr<lanelet::projection::UtmProjector> projector_;

  mutable std::vector<LaneInfo> cache_;  // results of findNearestLane()
  std::optional<FixedStartPosition> default_start_;

  /** Build a LaneInfo from a resolved lanelet (centroid + yaw). */
  [[nodiscard]] LaneInfo makeLaneInfo(const lanelet::ConstLanelet& ll) const;

  /** Yaw angle (radians) from the centre-line direction. */
  static double laneletYaw(const lanelet::ConstLanelet& ll);

  /** Quaternion {qz, qw} for a pure-Z rotation. */
  static std::pair<double, double> yawToQuat(double yaw_rad);
};

}  // namespace autoware_agent

#endif  // VEHICLE_AUTOWARE_AGENT_LANELETMAP_H
