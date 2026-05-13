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

#include "LaneletMap.h"

#include "Config.h"

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/LaneletMap.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_matching/LaneletMatching.h>
#include <lanelet2_routing/Route.h>
#include <spdlog/spdlog.h>

namespace fs = std::filesystem;
namespace autoware_agent {

LaneletMap::LaneletMap(const std::string& osm_path, double origin_lat, double origin_lon,
                       double local_offset_x, double local_offset_y)
  : origin_lat_(origin_lat)
  , origin_lon_(origin_lon)
  , offset_x_(local_offset_x)
  , offset_y_(local_offset_y) {
  std::string const full_path = resolveOsmPath(osm_path);
  if (!fs::exists(full_path)) {
    throw std::runtime_error("[LaneletMap] File not found: " + full_path);
  }

  // The UTM projector uses the map's WGS-84 origin so that all projected
  // (x,y) values are in the same metric frame as Autoware's local map.
  lanelet::Origin const ll_origin{{origin_lat_, origin_lon_}};
  projector_ = std::make_shared<lanelet::projection::UtmProjector>(ll_origin);

  lanelet::ErrorMessages errors;
  map_ = lanelet::load(full_path, *projector_, &errors);

  for (const auto& e : errors) {
    spdlog::warn("[LaneletMap] Load warning: {}", e);
  }

  if (!map_ || map_->laneletLayer.empty()) {
    throw std::runtime_error("[LaneletMap] Map has no lanelets: " + full_path);
  }

  spdlog::info("[LaneletMap] Loaded {} lanelets from {}", map_->laneletLayer.size(), full_path);
  traffic_rules_ = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules_);
  passable_map_ = routing_graph_->passableSubmap();
}

bool LaneletMap::isLoaded() const {
  return map_ && !map_->laneletLayer.empty();
}

size_t LaneletMap::getLaneletCount() const {
  return map_ ? map_->laneletLayer.size() : 0u;
}

LocalCoordinate LaneletMap::gpsToLocal(const GPSCoordinate& gps) const {
  GeographicLib::LocalCartesian proj(origin_lat_, origin_lon_, 0.0);
  double x{}, y{}, z{};
  proj.Forward(gps.latitude, gps.longitude, 0.0, x, y, z);
  return {offset_x_ + x, offset_y_ + y, 0.0};
}

bool LaneletMap::canRoute(int64_t from_lane_id, int64_t to_lane_id) const {
  if (!routing_graph_)
    return true;

  if (!map_->laneletLayer.exists(static_cast<lanelet::Id>(from_lane_id)) ||
      !map_->laneletLayer.exists(static_cast<lanelet::Id>(to_lane_id)))
    return false;

  auto from_ll = map_->laneletLayer.get(static_cast<lanelet::Id>(from_lane_id));
  auto to_ll = map_->laneletLayer.get(static_cast<lanelet::Id>(to_lane_id));

  auto route = routing_graph_->getRoute(from_ll, to_ll);
  return route.has_value();
}

GPSCoordinate LaneletMap::localToGps(const LocalCoordinate& local) const {
  GeographicLib::LocalCartesian proj(origin_lat_, origin_lon_, 0.0);
  double lat{}, lon{}, alt{};
  proj.Reverse(local.x - offset_x_, local.y - offset_y_, local.z, lat, lon, alt);
  return {lat, lon};
}

std::vector<const LaneInfo*> LaneletMap::findNearestLanes(const GPSCoordinate& gps,
                                                          unsigned max_candidates) const {
  if (!isLoaded())
    return {};

  // Query point in Lanelet2's internal frame (without our local offset)
  LocalCoordinate lc = gpsToLocal(gps);
  lanelet::BasicPoint2d query(lc.x - offset_x_, lc.y - offset_y_);

  // R-tree query on passable submap — already excludes non-vehicle lanelets
  auto nearest = lanelet::geometry::findNearest(passable_map_->laneletLayer, query, max_candidates);

  std::vector<const LaneInfo*> result;
  result.reserve(nearest.size());
  for (auto& [dist, ll] : nearest) {
    // Skip if traffic rules say this participant can't pass it
    if (!traffic_rules_->canPass(ll))
      continue;
    result.push_back(ensureCached(ll.id()));
  }
  return result;
}

const LaneInfo* LaneletMap::findNearestLane(const GPSCoordinate& gps) const {
  auto ranked = findNearestLanes(gps, 1);
  return ranked.empty() ? nullptr : ranked[0];
}

const LaneInfo* LaneletMap::findNearestRoutableLane(const GPSCoordinate& gps) const {
  return findNearestLaneImpl(gps, /*need_following=*/true, /*need_previous=*/false);
}

const LaneInfo* LaneletMap::findNearestReachableLane(const GPSCoordinate& gps) const {
  return findNearestLaneImpl(gps, /*need_following=*/false, /*need_previous=*/true);
}

const LaneInfo* LaneletMap::findNearestLaneImpl(const GPSCoordinate& gps, bool need_following,
                                                bool need_previous) const {
  if (!isLoaded())
    return nullptr;

  LocalCoordinate lc = gpsToLocal(gps);
  lanelet::BasicPoint2d query(lc.x - offset_x_, lc.y - offset_y_);

  constexpr unsigned kCandidates = 20;
  auto nearest = lanelet::geometry::findNearest(passable_map_->laneletLayer, query, kCandidates);

  if (nearest.empty()) {
    spdlog::warn("[LaneletMap] findNearest: no passable lanelets near ({:.6f},{:.6f})",
                 gps.latitude, gps.longitude);
    return nullptr;
  }

  lanelet::matching::Object2d obj;
  obj.pose.translation() = query;
  obj.pose.linear() = Eigen::Rotation2Dd(0).toRotationMatrix();  // heading unknown

  for (auto& [dist, ll] : nearest) {
    // Skip if routing connectivity not satisfied
    if (need_following && routing_graph_->following(ll).empty())
      continue;
    if (need_previous && routing_graph_->previous(ll).empty())
      continue;

    // Skip if traffic rules say this lanelet isn't passable for our participant
    if (!traffic_rules_->canPass(ll))
      continue;

    lanelet::Id id = ll.id();
    spdlog::debug(
      "[LaneletMap] GPS ({:.6f},{:.6f}) → lanelet {} dist={:.1f}m "
      "(following={} previous={})",
      gps.latitude, gps.longitude, id, dist, routing_graph_->following(ll).size(),
      routing_graph_->previous(ll).size());
    return ensureCached(id);
  }

  // Fallback: first passable lanelet regardless of connectivity
  spdlog::warn("[LaneletMap] No routable lanelet near ({:.6f},{:.6f}) — using nearest passable",
               gps.latitude, gps.longitude);
  return ensureCached(nearest.front().second.id());
}

const LaneInfo* LaneletMap::getLaneById(int64_t lane_id) const {
  for (const auto& cached : cache_) {
    if (cached.lane_id == lane_id)
      return &cached;
  }
  if (!map_->laneletLayer.exists(static_cast<lanelet::Id>(lane_id)))
    return nullptr;

  lanelet::ConstLanelet const ll = map_->laneletLayer.get(static_cast<lanelet::Id>(lane_id));
  cache_.emplace_back(makeLaneInfo(ll));
  return &cache_.back();
}

const LaneInfo* LaneletMap::ensureCached(lanelet::Id id) const {
  for (const auto& c : cache_)
    if (c.lane_id == static_cast<int64_t>(id))
      return &c;
  auto ll = map_->laneletLayer.get(id);
  cache_.emplace_back(makeLaneInfo(ll));
  return &cache_.back();
}

void LaneletMap::setDefaultStart(const std::string& name, const GPSCoordinate& gps) {
  FixedStartPosition pos;
  pos.name = name;
  pos.gps = gps;

  // Resolve the lanelet immediately so the rest of the code gets stable
  // local coordinates without an extra lookup later.
  const LaneInfo* lane = findNearestLane(gps);
  if (lane) {
    pos.local = lane->local;
    pos.orientation = lane->orientation;
  } else {
    // Fallback: raw projection (no nearest-lanelet snap).
    pos.local = gpsToLocal(gps);
    spdlog::warn(
      "[LaneletMap] setDefaultStart: no lanelet found near "
      "({:.6f},{:.6f}) — using raw projection",
      gps.latitude, gps.longitude);
  }

  default_start_ = pos;
  spdlog::info("[LaneletMap] Default start '{}' → local ({:.2f},{:.2f})", name, pos.local.x,
               pos.local.y);
}

const FixedStartPosition* LaneletMap::getDefaultStart() const {
  return default_start_.has_value() ? &default_start_.value() : nullptr;
}

std::string LaneletMap::resolveOsmPath(const std::string& filename) {
  fs::path input(filename);
  if (input.is_absolute() && fs::exists(input))
    return input.string();

  // Relative path: probe the same directories RouteConfig used.
  for (const auto& base : {autoware_agent::TEST_MAP_DIR, autoware_agent::SRC_MAP_DIR,
                           autoware_agent::INSTALL_MAP_DIR}) {
    fs::path candidate = fs::path(base) / filename;
    if (fs::exists(candidate))
      return candidate.string();
  }

  // Last resort: return as-is and let the caller surface the error.
  return filename;
}

LaneInfo LaneletMap::makeLaneInfo(const lanelet::ConstLanelet& ll) const {
  const auto& cl = ll.centerline();
  const auto& mid = cl[cl.size() / 2];

  double const lx = offset_x_ + mid.x();
  double const ly = offset_y_ + mid.y();
  double const yaw_rad = laneletYaw(ll);
  auto const [qz, qw] = yawToQuat(yaw_rad);

  LaneInfo info;
  info.lane_id = static_cast<int64_t>(ll.id());
  info.local.x = lx;
  info.local.y = ly;
  info.local.z = 0.0;
  info.gps = localToGps({lx, ly, 0.0});
  info.orientation.x = 0.0;
  info.orientation.y = 0.0;
  info.orientation.z = qz;
  info.orientation.w = qw;
  info.orientation.yaw_degrees = yaw_rad * 180.0 / M_PI;
  return info;
}

double LaneletMap::laneletYaw(const lanelet::ConstLanelet& ll) {
  const auto& cl = ll.centerline();
  if (cl.size() < 2)
    return 0.0;
  const auto& p0 = cl.front();
  const auto& p1 = cl.back();
  return std::atan2(p1.y() - p0.y(), p1.x() - p0.x());
}

std::pair<double, double> LaneletMap::yawToQuat(double yaw_rad) {
  double const h = yaw_rad * 0.5;
  return {std::sin(h), std::cos(h)};  // {qz, qw}
}

}  // namespace autoware_agent