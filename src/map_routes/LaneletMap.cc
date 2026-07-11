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

#include "Config.h"  // TEST_MAP_DIR / SRC_MAP_DIR / INSTALL_MAP_DIR

#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/primitives/Lanelet.h>
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
  projector_ = std::make_shared<lanelet::projection::MGRSProjector>();
  projector_->setMGRSCode(lanelet::GPSPoint{origin_lat_, origin_lon_});

  lanelet::ErrorMessages errors;
  map_ = lanelet::load(full_path, *projector_, &errors);

  // added to solve the problem of the cache growing indefinitely when the map is large and
  // findNearestLane is called many times with different GPS coordinates. By reserving space in the
  // cache vector upfront, we can avoid unnecessary reallocations and improve performance.
  cache_.reserve(map_->laneletLayer.size());

  for (const auto& e : errors) {
    spdlog::warn("[LaneletMap] Load warning: {}", e);
  }

  if (!map_ || map_->laneletLayer.empty()) {
    throw std::runtime_error("[LaneletMap] Map has no lanelets: " + full_path);
  }

  spdlog::info("[LaneletMap] Loaded {} lanelets from {}", map_->laneletLayer.size(), full_path);

  // Build routing graph
  auto traffic_rules = lanelet::traffic_rules::TrafficRulesFactory::create(
    lanelet::Locations::Germany, lanelet::Participants::Vehicle);
  routing_graph_ = lanelet::routing::RoutingGraph::build(*map_, *traffic_rules);
  spdlog::info("[LaneletMap] Routing graph built — {} lanelets", map_->laneletLayer.size());
}

bool LaneletMap::isLoaded() const {
  return map_ && !map_->laneletLayer.empty();
}

size_t LaneletMap::getLaneletCount() const {
  return map_ ? map_->laneletLayer.size() : 0u;
}

LocalCoordinate LaneletMap::gpsToLocal(const GPSCoordinate& gps) const {
  lanelet::BasicPoint3d pt = projector_->forward({gps.latitude, gps.longitude, 0.0});
  return {pt.x(), pt.y(), pt.z()};
}

GPSCoordinate LaneletMap::localToGps(const LocalCoordinate& local) const {
  lanelet::GPSPoint gps_pt = projector_->reverse({local.x, local.y, local.z});
  return {gps_pt.lat, gps_pt.lon};
}

const LaneInfo* LaneletMap::findNearestLane(const GPSCoordinate& gps) const {
  if (!isLoaded())
    return nullptr;

  spdlog::info("[LaneletMap] findNearestLane called with GPS ({:.6f},{:.6f})", gps.latitude,
               gps.longitude);

  // Convert the query GPS to the local map frame for distance comparisons.
  LocalCoordinate const target = gpsToLocal(gps);
  spdlog::info(
    "[LaneletMap] findNearestLane called with local ({:.6f},{:.6f}) gps_lat={:.10f} "
    "gps_lon={:.10f}",
    target.x, target.y, gps.latitude, gps.longitude);

  double min_dist = std::numeric_limits<double>::max();
  lanelet::Id best_id = lanelet::InvalId;

  for (const auto& ll : map_->laneletLayer) {
    // Skip non-drivable lanelets (crosswalks, walkways)
    auto subtype = ll.attributes().find("subtype");
    if (subtype != ll.attributes().end()) {
      const std::string& st = subtype->second.value();
      if (st == "crosswalk" || st == "walkway" || st == "stairs" || st == "pedestrian") {
        continue;
      }
    }

    const auto& cl = ll.centerline();
    if (cl.empty())
      continue;

    // Representative point: midpoint of the centre-line.
    const auto& mid = cl[cl.size() / 2];

    // mid.x() / mid.y() are in UTM relative to the projector origin;
    // apply the same local offset used in gpsToLocal().
    double const lx = offset_x_ + mid.x();
    double const ly = offset_y_ + mid.y();
    double const dx = lx - target.x;
    double const dy = ly - target.y;
    double const dist = std::sqrt(dx * dx + dy * dy);

    if (dist < min_dist) {
      min_dist = dist;
      best_id = ll.id();
    }
  }

  if (best_id == lanelet::InvalId)
    return nullptr;

  // Check the cache first — avoids repeated construction for the same lanelet.
  for (const auto& cached : cache_) {
    if (cached.lane_id == static_cast<int64_t>(best_id)) {
      spdlog::warn(
        "[DEBUG] findNearestLane returning CACHED entry for lane {} local=({:.3f},{:.3f})", best_id,
        cached.local.x, cached.local.y);
      return &cached;
    }
  }

  lanelet::ConstLanelet const ll = map_->laneletLayer.get(best_id);
  cache_.emplace_back(makeLaneInfo(ll, gps));

  spdlog::debug(
    "[LaneletMap] GPS ({:.6f},{:.6f}) → lanelet {} "
    "local ({:.2f},{:.2f}) dist={:.1f}m",
    gps.latitude, gps.longitude, best_id, cache_.back().local.x, cache_.back().local.y, min_dist);

  // ADD THIS — print the lanelet type
  auto subtype = ll.attributes().find("subtype");
  auto type = ll.attributes().find("type");
  spdlog::info(
    "[LaneletMap] GPS ({:.6f},{:.6f}) → lanelet {} "
    "type={} subtype={} dist={:.1f}m",
    gps.latitude, gps.longitude, best_id,
    (type != ll.attributes().end() ? type->second.value() : "unknown"),
    (subtype != ll.attributes().end() ? subtype->second.value() : "unknown"), min_dist);

  return &cache_.back();
}

const LaneInfo* LaneletMap::getLaneById(int64_t lane_id) const {
  for (const auto& cached : cache_) {
    if (cached.lane_id == lane_id)
      return &cached;
  }
  if (!map_->laneletLayer.exists(static_cast<lanelet::Id>(lane_id)))
    return nullptr;

  lanelet::ConstLanelet const ll = map_->laneletLayer.get(static_cast<lanelet::Id>(lane_id));
  const auto& cl = ll.centerline();
  const auto& mid = cl[cl.size() / 2];
  auto mid_gps = localToGps({mid.x(), mid.y(), mid.z()});

  cache_.emplace_back(makeLaneInfo(ll, mid_gps));

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

// ── Path resolution ───────────────────────────────────────────────────────────

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

// ── Private helpers ───────────────────────────────────────────────────────────

LaneInfo LaneletMap::makeLaneInfo(const lanelet::ConstLanelet& ll, const GPSCoordinate& gps) const {
  LocalCoordinate projected = projectOntoLaneCenterline(gps, ll.id());
  double const yaw_rad = laneletYaw(ll);
  auto const [qz, qw] = yawToQuat(yaw_rad);

  LaneInfo info;
  info.lane_id = ll.id();
  info.gps = localToGps(projected);
  info.local = projected;
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

// added for debugging route connectivity between two lanelets
void LaneletMap::debugRouteConnectivity(int64_t from_id, int64_t to_id) const {
  if (!routing_graph_) {
    spdlog::error("[LaneletMap] routing graph not built");
    return;
  }

  auto from_it = map_->laneletLayer.find(from_id);
  auto to_it = map_->laneletLayer.find(to_id);

  if (from_it == map_->laneletLayer.end() || to_it == map_->laneletLayer.end()) {
    spdlog::error("[LaneletMap] Invalid lanelet IDs: {} or {}", from_id, to_id);
    return;
  }

  lanelet::ConstLanelet from_ll = *from_it;
  lanelet::ConstLanelet to_ll = *to_it;

  auto route = routing_graph_->getRoute(from_ll, to_ll);
  if (!route) {
    spdlog::error("[LaneletMap] NO ROUTE from {} to {} — lanelets disconnected", from_id, to_id);
  } else {
    spdlog::info("[LaneletMap] Route exists from {} to {} ({} lanelets in shortest path)", from_id,
                 to_id, route->shortestPath().size());
  }

  auto followers = routing_graph_->following(from_ll);
  spdlog::info("[LaneletMap] Lane {} has {} followers:", from_id, followers.size());
  for (const auto& f : followers)
    spdlog::info("  → {}", f.id());

  auto prev = routing_graph_->previous(to_ll);
  spdlog::info("[LaneletMap] Lane {} has {} predecessors:", to_id, prev.size());
  for (const auto& p : prev)
    spdlog::info("  ← {}", p.id());

  auto reachable = routing_graph_->reachableSet(from_ll, 200.0);
  spdlog::info("[LaneletMap] Reachable from {} within 200m: {} lanelets", from_id,
               reachable.size());
  bool found = false;
  for (const auto& ll : reachable)
    if (ll.id() == to_id) {
      found = true;
      break;
    }
  spdlog::info("[LaneletMap] Lane {} {} within 200m reachable set of {}", to_id,
               found ? "IS" : "IS NOT", from_id);
}

// LaneletMap.cpp
const LaneInfo* LaneletMap::findNearestConnectedLane(const GPSCoordinate& gps,
                                                     lanelet::Id reference_id,
                                                     bool must_be_reachable_from_ref) const {
  if (!isLoaded() || !routing_graph_)
    return nullptr;

  LocalCoordinate const target = gpsToLocal(gps);

  spdlog::info("[gps to local conversion] x={:.6f} y={:.6f}", target.x, target.y);

  lanelet::ConstLanelet reference_ll = map_->laneletLayer.get(reference_id);

  // Get the full set of lanelets reachable from the reference,
  // within a generous radius  8km
  lanelet::ConstLanelets candidates;
  if (must_be_reachable_from_ref) {
    candidates = routing_graph_->reachableSet(reference_ll, 8000.0);
  } else {
    // reachable TO reference — i.e. predecessors transitively
    // lanelet2 doesn't have a direct "reverse reachableSet", so build it
    // by checking each lanelet's reachableSet for membership of reference
    for (const auto& ll : map_->laneletLayer) {
      auto reach = routing_graph_->reachableSet(ll, 8000.0);
      for (const auto& r : reach) {
        if (r.id() == reference_id) {
          candidates.push_back(ll);
          break;
        }
      }
    }
  }

  spdlog::error(
    "[debug inside findNearestConnectedLane] GPS ({:.6f},{:.6f}) local ({:.6f},{:.6f}) reference "
    "lane {} candidates {}",
    gps.latitude, gps.longitude, target.x, target.y, reference_id, candidates.size());

  double min_dist = std::numeric_limits<double>::max();
  lanelet::Id best_id = lanelet::InvalId;

  for (const auto& ll : candidates) {
    auto subtype = ll.attributes().find("subtype");
    if (subtype != ll.attributes().end()) {
      const std::string& st = subtype->second.value();
      if (st == "crosswalk" || st == "walkway" || st == "stairs" || st == "pedestrian")
        continue;
    }
    const auto& cl = ll.centerline();
    if (cl.empty())
      continue;
    const auto& mid = cl[cl.size() / 2];
    double const lx = offset_x_ + mid.x();
    double const ly = offset_y_ + mid.y();
    double const dx = lx - target.x;
    double const dy = ly - target.y;
    double const dist = std::sqrt(dx * dx + dy * dy);
    if (dist < min_dist) {
      min_dist = dist;
      best_id = ll.id();
    }
  }

  if (best_id == lanelet::InvalId) {
    spdlog::warn(
      "[LaneletMap] No connected candidate found near ({:.6f},{:.6f}) "
      "relative to reference lane {}",
      gps.latitude, gps.longitude, reference_id);
    return nullptr;
  }

  for (const auto& cached : cache_) {
    if (cached.lane_id == static_cast<int64_t>(best_id))
      return &cached;
  }

  lanelet::ConstLanelet const ll = map_->laneletLayer.get(best_id);
  cache_.emplace_back(makeLaneInfo(ll, gps));
  spdlog::info("[LaneletMap] GPS ({:.6f},{:.6f}) → connected lanelet {} dist={:.1f}m", gps.latitude,
               gps.longitude, best_id, min_dist);

  return &cache_.back();
}

// Add a new function to LaneletMap that projects a GPS point onto
// its resolved lane's ACTUAL centerline, instead of returning the midpoint:
LocalCoordinate LaneletMap::projectOntoLaneCenterline(const GPSCoordinate& gps,
                                                      lanelet::Id lane_id) const {
  lanelet::ConstLanelet ll = map_->laneletLayer.get(lane_id);
  const auto& cl = ll.centerline();

  LocalCoordinate target = gpsToLocal(gps);

  spdlog::info("[DEBUG projectOntoLaneCenterline] lane={} target=({:.3f},{:.3f}) cl.size()={}",
               lane_id, target.x, target.y, cl.size());

  // Find the closest point ON the centerline (not just the midpoint)
  double min_dist = std::numeric_limits<double>::max();
  lanelet::BasicPoint3d best_point;

  for (size_t i = 0; i + 1 < cl.size(); ++i) {
    // project target onto each centerline segment, find closest segment
    const auto& p0 = cl[i];
    const auto& p1 = cl[i + 1];
    // (standard point-to-segment projection math)
    double dx = p1.x() - p0.x();
    double dy = p1.y() - p0.y();
    double seg_len2 = dx * dx + dy * dy;
    double t =
      seg_len2 > 0
        ? std::clamp(((target.x - p0.x()) * dx + (target.y - p0.y()) * dy) / seg_len2, 0.0, 1.0)
        : 0.0;
    double px = p0.x() + t * dx;
    double py = p0.y() + t * dy;
    double dist = std::sqrt((target.x - px) * (target.x - px) + (target.y - py) * (target.y - py));
    if (dist < min_dist) {
      min_dist = dist;
      best_point = lanelet::BasicPoint3d(px, py, p0.z());
    }
  }

  spdlog::info("[DEBUG projectOntoLaneCenterline] result=({:.3f},{:.3f},{:.3f}) min_dist={:.3f}",
               best_point.x(), best_point.y(), best_point.z(), min_dist);

  return {best_point.x(), best_point.y(), best_point.z()};
}

void LaneletMap::debugConnectedComponents() const {
  if (!routing_graph_) {
    spdlog::error("[LaneletMap] routing graph not built");
    return;
  }

  std::set<lanelet::Id> visited;
  std::vector<std::vector<lanelet::Id>> components;

  for (const auto& ll : map_->laneletLayer) {
    auto subtype = ll.attributes().find("subtype");
    if (subtype != ll.attributes().end()) {
      const std::string& st = subtype->second.value();
      if (st == "crosswalk" || st == "walkway" || st == "stairs" || st == "pedestrian")
        continue;
    }

    if (visited.count(ll.id()))
      continue;

    // BFS/flood-fill: get everything mutually reachable (treat as undirected
    // by checking reachability both ways) starting from this lanelet.
    std::vector<lanelet::Id> component;
    std::queue<lanelet::ConstLanelet> to_visit;
    to_visit.push(ll);
    visited.insert(ll.id());

    while (!to_visit.empty()) {
      auto current = to_visit.front();
      to_visit.pop();
      component.push_back(current.id());

      // Forward neighbors
      for (const auto& f : routing_graph_->following(current)) {
        if (!visited.count(f.id())) {
          visited.insert(f.id());
          to_visit.push(f);
        }
      }
      // Backward neighbors
      for (const auto& p : routing_graph_->previous(current)) {
        if (!visited.count(p.id())) {
          visited.insert(p.id());
          to_visit.push(p);
        }
      }
      // Also check side-by-side (left/right) lane relationships, since a
      // lanelet could be connected via lane-change rather than follow/prev
      auto left = routing_graph_->left(current);
      if (left && !visited.count(left->id())) {
        visited.insert(left->id());
        to_visit.push(*left);
      }
      auto right = routing_graph_->right(current);
      if (right && !visited.count(right->id())) {
        visited.insert(right->id());
        to_visit.push(*right);
      }
    }

    components.push_back(component);
  }

  // Sort components by size, descending — the main road network should be
  // by far the largest; isolated islands will be tiny.
  std::sort(components.begin(), components.end(),
            [](const auto& a, const auto& b) { return a.size() > b.size(); });

  spdlog::info("[LaneletMap] ===== Connected components: {} total =====", components.size());
  for (size_t i = 0; i < components.size(); ++i) {
    spdlog::info("[LaneletMap] component #{}: {} lanelets", i, components[i].size());
    if (components[i].size() <= 30) {
      // Print full membership for small components — likely the isolated islands
      std::string ids;
      for (auto id : components[i])
        ids += std::to_string(id) + " ";
      spdlog::info("[LaneletMap]   members: {}", ids);
    }
  }
}

void LaneletMap::debugFindBestStartingLane() const {
  if (!routing_graph_) {
    spdlog::error("[LaneletMap] routing graph not built");
    return;
  }

  std::vector<std::pair<size_t, lanelet::Id>> reach_counts;

  for (const auto& ll : map_->laneletLayer) {
    auto subtype = ll.attributes().find("subtype");
    if (subtype != ll.attributes().end()) {
      const std::string& st = subtype->second.value();
      if (st == "crosswalk" || st == "walkway" || st == "stairs" || st == "pedestrian")
        continue;
    }
    // reachableSet with a very large radius to approximate "everything reachable
    // at all", regardless of distance — just checking connectivity breadth.
    auto reach = routing_graph_->reachableSet(ll, 1000000.0);
    reach_counts.push_back({reach.size(), ll.id()});
  }

  std::sort(reach_counts.begin(), reach_counts.end(),
            [](const auto& a, const auto& b) { return a.first > b.first; });

  spdlog::info("[LaneletMap] ===== Top 10 lanelets by forward-reachable count =====");
  for (size_t i = 0; i < std::min<size_t>(10, reach_counts.size()); ++i) {
    spdlog::info("[LaneletMap] #{}: lane={} reaches {} lanelets", i, reach_counts[i].second,
                 reach_counts[i].first);
  }

  spdlog::info("[LaneletMap] ===== Bottom 10 (most isolated) =====");
  for (size_t i = 0; i < std::min<size_t>(10, reach_counts.size()); ++i) {
    size_t idx = reach_counts.size() - 1 - i;
    spdlog::info("[LaneletMap] lane={} reaches only {} lanelets", reach_counts[idx].second,
                 reach_counts[idx].first);
  }
}

}  // namespace autoware_agent