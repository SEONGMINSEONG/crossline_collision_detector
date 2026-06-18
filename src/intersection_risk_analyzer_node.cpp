// Copyright 2026 SEONGMINSEONG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "crossline_collision_detector/intersection_risk_analyzer_node.hpp"

#include <lanelet2_core/geometry/Lanelet.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace crossline_collision_detector
{

// 노드 초기화: 파라미터 선언, 입출력 토픽 연결
IntersectionRiskAnalyzerNode::IntersectionRiskAnalyzerNode(const rclcpp::NodeOptions & options)
: Node("intersection_risk_analyzer", options)
{
  using std::placeholders::_1;

  params_.eta_gap_threshold = declare_parameter<double>("eta_gap_threshold", 1.5);
  params_.prediction_horizon_m = declare_parameter<double>("prediction_horizon_m", 25.0);
  params_.path_overlap_distance_threshold =
    declare_parameter<double>("path_overlap_distance_threshold", 2.5);
  params_.min_vehicle_speed = declare_parameter<double>("min_vehicle_speed", 2.0);
  params_.use_consecutive_validation =
    declare_parameter<bool>("use_consecutive_validation", true);
  params_.consecutive_count_threshold = declare_parameter<int>("consecutive_count_threshold", 3);
  params_.use_priority_lock = declare_parameter<bool>("use_priority_lock", true);
  params_.priority_unlock_count_threshold =
    declare_parameter<int>("priority_unlock_count_threshold", 3);
  params_.warning_hold_duration_sec = declare_parameter<double>("warning_hold_duration_sec", 3.0);
  params_.conflict_zone_lanelet_ids =
    declare_parameter<std::vector<std::int64_t>>(
    "conflict_zone_lanelet_ids", std::vector<std::int64_t>{});
  params_.approach_ids =
    declare_parameter<std::vector<std::string>>("approach_ids", std::vector<std::string>{"A", "B"});
  params_.allowed_conflicting_approach_pairs =
    declare_parameter<std::vector<std::string>>(
    "allowed_conflicting_approach_pairs", std::vector<std::string>{});
  params_.stable_id_grid_size = declare_parameter<double>("stable_id_grid_size", 3.0);
  params_.traffic_rule_location =
    declare_parameter<std::string>("traffic_rule_location", lanelet::Locations::Germany);
  params_.traffic_rule_participant =
    declare_parameter<std::string>("traffic_rule_participant", "vehicle");

  for (const auto & approach_id : params_.approach_ids) {
    const auto lanelet_param = "approach_" + approach_id + "_lanelet_ids";
    approach_lanelet_id_map_[approach_id] =
      declare_parameter<std::vector<std::int64_t>>(
      lanelet_param, std::vector<std::int64_t>{});

    const auto label_param = "approach_" + approach_id + "_label";
    approach_label_map_[approach_id] = declare_parameter<std::string>(label_param, approach_id);

    const auto maneuver_param = "approach_" + approach_id + "_maneuver";
    const auto right_of_param = "approach_" + approach_id + "_right_of";
    ApproachPriorityRule rule;
    rule.maneuver = declare_parameter<std::string>(maneuver_param, "straight");
    rule.right_of = declare_parameter<std::string>(right_of_param, "");
    approach_priority_rules_[approach_id] = rule;
  }

  sub_objects_ = create_subscription<PredictedObjects>(
    "~/input/objects", rclcpp::QoS{1},
    std::bind(&IntersectionRiskAnalyzerNode::onObjects, this, _1));

  sub_map_ = create_subscription<HADMapBin>(
    "~/input/vector_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&IntersectionRiskAnalyzerNode::onMap, this, _1));

  pub_status_ = create_publisher<IntersectionRiskStatus>("~/output/status", 1);
  pub_approach_warnings_ =
    create_publisher<ApproachWarningArray>("~/output/approach_warnings", 1);
  pub_conflicting_pairs_ =
    create_publisher<ConflictingPairArray>("~/output/conflicting_pairs", 1);
  pub_debug_markers_ =
    create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 1);
}

// Lanelet map을 수신하면 내부 맵과 routing graph를 준비한다.
void IntersectionRiskAnalyzerNode::onMap(const HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_);

  std::string location = lanelet::Locations::Germany;
  if (params_.traffic_rule_location == "germany" || params_.traffic_rule_location == "de") {
    location = lanelet::Locations::Germany;
  } else {
    location = params_.traffic_rule_location;
  }

  std::string participant = lanelet::Participants::Vehicle;
  if (params_.traffic_rule_participant == "vehicle") {
    participant = lanelet::Participants::Vehicle;
  } else if (params_.traffic_rule_participant == "pedestrian") {
    participant = lanelet::Participants::Pedestrian;
  } else if (params_.traffic_rule_participant == "bicycle") {
    participant = lanelet::Participants::Bicycle;
  }

  traffic_rules_ptr_ =
    lanelet::traffic_rules::TrafficRulesFactory::create(location, participant);
  routing_graph_ptr_ =
    lanelet::routing::RoutingGraph::build(*lanelet_map_ptr_, *traffic_rules_ptr_);
}

// 객체 프레임을 받을 때마다 zone 진입 가능성과 ETA 차이를 계산한다.
void IntersectionRiskAnalyzerNode::onObjects(const PredictedObjects::ConstSharedPtr msg)
{
  if (!lanelet_map_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Map not loaded yet");
    return;
  }

  const auto start_time = std::chrono::high_resolution_clock::now();
  const auto zone_lanelets = getConflictZoneLanelets();

  // 현재 프레임에서 접근로가 분류된 차량과 위험 차량 쌍을 모은다.
  std::vector<TrackedObjectInfo> tracked_objects;
  std::vector<PairRiskResult> risky_pairs;
  std::vector<std::string> object_debug_summaries;

  // 1. 차량별 전처리: 속도 필터, lanelet 매칭, 접근로 판별, predicted path 확인
  for (const auto & object : msg->objects) {
    const double speed = getObjectSpeed(object);
    if (speed < params_.min_vehicle_speed) {
      continue;
    }

    const auto matched_lanelet = findClosestLanelet(object);
    if (!matched_lanelet) {
      continue;
    }

    TrackedObjectInfo info;
    info.object_id = getStableObjectId(object);
    info.approach_id = classifyApproach(object, matched_lanelet.get());
    info.maneuver = approach_priority_rules_.count(info.approach_id) ?
      approach_priority_rules_.at(info.approach_id).maneuver : "unknown";
    info.speed = speed;
    info.eta_to_zone = calculateEtaToConflictZone(object, zone_lanelets);
    info.reaches_conflict_zone = reachesConflictZone(object, zone_lanelets);
    info.object = object;

    std::ostringstream summary;
    summary << "id=" << info.object_id << " lanelet=" << matched_lanelet->id()
            << " approach=" << info.approach_id << " speed=" << std::fixed
            << std::setprecision(2) << info.speed << " reaches_zone="
            << (info.reaches_conflict_zone ? "true" : "false");
    if (std::isfinite(info.eta_to_zone)) {
      summary << " eta=" << std::fixed << std::setprecision(2) << info.eta_to_zone;
    } else {
      summary << " eta=inf";
    }
    object_debug_summaries.push_back(summary.str());

    if (info.approach_id != "unknown" && !info.object.kinematics.predicted_paths.empty()) {
      tracked_objects.push_back(info);
    }
  }

  if (!object_debug_summaries.empty()) {
    std::ostringstream debug_stream;
    debug_stream << "object classification: ";
    for (size_t i = 0; i < object_debug_summaries.size(); ++i) {
      if (i > 0) {
        debug_stream << " | ";
      }
      debug_stream << object_debug_summaries[i];
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000, "%s", debug_stream.str().c_str());
  }

  // 2. 차량 쌍별 위험 여부를 평가한다.
  std::unordered_set<std::string> seen_pair_keys;
  for (size_t i = 0; i < tracked_objects.size(); ++i) {
    for (size_t j = i + 1; j < tracked_objects.size(); ++j) {
      if (tracked_objects[i].approach_id == tracked_objects[j].approach_id) {
        continue;
      }
      if (!isApproachPairAllowed(
          tracked_objects[i].approach_id, tracked_objects[j].approach_id))
      {
        continue;
      }

      auto pair_result = evaluatePairRisk(tracked_objects[i], tracked_objects[j], zone_lanelets);
      const auto pair_key = makePairKey(tracked_objects[i].object_id, tracked_objects[j].object_id);
      seen_pair_keys.insert(pair_key);
      auto & history = pair_history_[pair_key];

      if (pair_result.risky) {
        history.consecutive_risk_count++;
        history.consecutive_miss_count = 0;
        updatePriorityLock(pair_result, history);
      } else {
        history.consecutive_risk_count = 0;
        history.consecutive_miss_count++;
        if (history.consecutive_miss_count >= params_.priority_unlock_count_threshold) {
          history.has_priority_lock = false;
          history.locked_priority_object_id.clear();
        }
      }

      const bool is_confirmed = !params_.use_consecutive_validation ||
        history.consecutive_risk_count >= params_.consecutive_count_threshold;

      if (pair_result.risky && is_confirmed) {
        risky_pairs.push_back(pair_result);
      }
    }
  }

  for (auto it = pair_history_.begin(); it != pair_history_.end(); ) {
    if (seen_pair_keys.count(it->first) == 0) {
      it->second.consecutive_risk_count = 0;
      it->second.consecutive_miss_count++;
      if (it->second.consecutive_miss_count >= params_.priority_unlock_count_threshold) {
        it = pair_history_.erase(it);
        continue;
      }
    }
    ++it;
  }

  double min_eta_gap = std::numeric_limits<double>::infinity();
  for (const auto & pair : risky_pairs) {
    min_eta_gap = std::min(min_eta_gap, pair.eta_gap);
  }
  if (!std::isfinite(min_eta_gap)) {
    min_eta_gap = -1.0;
  }

  const auto end_time = std::chrono::high_resolution_clock::now();
  const auto processing_time_us =
    std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
  const double processing_time_ms = processing_time_us / 1000.0;

  // 3. 요약 상태, 접근로 경고, 위험 차량 쌍, 시각화 정보를 출력한다.
  publishStatus(
    msg->header, tracked_objects.size(), risky_pairs.size(), min_eta_gap, processing_time_ms);
  publishApproachWarnings(msg->header, risky_pairs);
  publishConflictingPairs(msg->header, risky_pairs);
  pub_debug_markers_->publish(createDebugMarkers(msg->header, tracked_objects, risky_pairs));
}

// 객체가 실제로 포함된 lanelet을 간단히 찾는다.
boost::optional<lanelet::ConstLanelet> IntersectionRiskAnalyzerNode::findClosestLanelet(
  const PredictedObject & object) const
{
  const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d search_point(pos.x, pos.y);
  const auto nearby_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 10);

  for (const auto & [dist, lanelet] : nearby_lanelets) {
    (void)dist;
    if (isConflictZoneLaneletId(lanelet.id())) {
      continue;
    }
    if (lanelet::geometry::inside(lanelet, search_point)) {
      return lanelet;
    }
  }

  return boost::none;
}

bool IntersectionRiskAnalyzerNode::isConflictZoneLaneletId(const lanelet::Id lanelet_id) const
{
  return std::find(
    params_.conflict_zone_lanelet_ids.begin(), params_.conflict_zone_lanelet_ids.end(),
    lanelet_id) != params_.conflict_zone_lanelet_ids.end();
}

// 객체의 xy 평면 속도 크기를 계산한다.
double IntersectionRiskAnalyzerNode::getObjectSpeed(const PredictedObject & object) const
{
  const auto & twist = object.kinematics.initial_twist_with_covariance.twist;
  return std::sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
}

// 위치를 3m 격자로 양자화해 간단한 stable ID를 만든다.
std::string IntersectionRiskAnalyzerNode::getStableObjectId(const PredictedObject & object) const
{
  const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
  bool has_non_zero_uuid = false;
  for (const auto byte : object.object_id.uuid) {
    if (byte != 0U) {
      has_non_zero_uuid = true;
      break;
    }
  }

  if (has_non_zero_uuid) {
    return uuidToString(object.object_id);
  }

  const int grid_x = static_cast<int>(std::round(pos.x / params_.stable_id_grid_size));
  const int grid_y = static_cast<int>(std::round(pos.y / params_.stable_id_grid_size));
  return "grid_" + std::to_string(grid_x) + "_" + std::to_string(grid_y);
}

std::string IntersectionRiskAnalyzerNode::uuidToString(
  const unique_identifier_msgs::msg::UUID & uuid) const
{
  std::ostringstream ss;
  for (size_t i = 0; i < uuid.uuid.size(); ++i) {
    ss << std::hex << std::setfill('0') << std::setw(2) << static_cast<int>(uuid.uuid[i]);
  }
  return ss.str();
}

// predicted path 중 하나라도 conflict zone에 들어가면 true를 반환한다.
bool IntersectionRiskAnalyzerNode::reachesConflictZone(
  const PredictedObject & object,
  const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const
{
  for (const auto & path : object.kinematics.predicted_paths) {
    for (const auto & pose : path.path) {
      lanelet::BasicPoint2d p(pose.position.x, pose.position.y);
      for (const auto & zone_lanelet : conflict_zone_lanelets) {
        if (lanelet::geometry::inside(zone_lanelet, p)) {
          return true;
        }
      }
    }
  }
  return false;
}

// predicted path 누적거리와 현재 속도를 이용해 zone 도달 ETA를 근사한다.
double IntersectionRiskAnalyzerNode::calculateEtaToConflictZone(
  const PredictedObject & object,
  const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const
{
  const double speed = getObjectSpeed(object);
  if (speed < 0.1) {
    return std::numeric_limits<double>::infinity();
  }

  for (const auto & path : object.kinematics.predicted_paths) {
    double accumulated_distance = 0.0;
    for (size_t i = 1; i < path.path.size(); ++i) {
      const auto & prev = path.path[i - 1].position;
      const auto & curr = path.path[i].position;

      const double dx = curr.x - prev.x;
      const double dy = curr.y - prev.y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);

      lanelet::BasicPoint2d p(curr.x, curr.y);
      for (const auto & zone_lanelet : conflict_zone_lanelets) {
        if (lanelet::geometry::inside(zone_lanelet, p)) {
          return accumulated_distance / speed;
        }
      }
    }
  }

  return std::numeric_limits<double>::infinity();
}

std::vector<IntersectionRiskAnalyzerNode::PathSample>
IntersectionRiskAnalyzerNode::samplePredictedPathWithinHorizon(
  const PredictedObject & object) const
{
  std::vector<PathSample> samples;
  const auto & start = object.kinematics.initial_pose_with_covariance.pose.position;
  const double horizon = std::max(params_.prediction_horizon_m, 0.0);

  for (const auto & path : object.kinematics.predicted_paths) {
    double accumulated_distance = 0.0;
    double previous_x = start.x;
    double previous_y = start.y;

    for (const auto & pose : path.path) {
      const double dx = pose.position.x - previous_x;
      const double dy = pose.position.y - previous_y;
      accumulated_distance += std::sqrt(dx * dx + dy * dy);

      if (accumulated_distance > horizon) {
        break;
      }

      samples.push_back(PathSample{pose.position.x, pose.position.y, accumulated_distance});
      previous_x = pose.position.x;
      previous_y = pose.position.y;
    }
  }

  return samples;
}

IntersectionRiskAnalyzerNode::PathOverlapResult IntersectionRiskAnalyzerNode::evaluatePathOverlap(
  const TrackedObjectInfo & object_a,
  const TrackedObjectInfo & object_b) const
{
  PathOverlapResult result;
  result.min_distance = std::numeric_limits<double>::infinity();
  result.eta_a = std::numeric_limits<double>::infinity();
  result.eta_b = std::numeric_limits<double>::infinity();
  result.eta_gap = std::numeric_limits<double>::infinity();

  const auto samples_a = samplePredictedPathWithinHorizon(object_a.object);
  const auto samples_b = samplePredictedPathWithinHorizon(object_b.object);
  if (samples_a.empty() || samples_b.empty()) {
    return result;
  }

  const double distance_threshold = std::max(params_.path_overlap_distance_threshold, 0.0);
  const double distance_threshold_sq = distance_threshold * distance_threshold;
  double min_distance_sq = std::numeric_limits<double>::infinity();

  for (const auto & sample_a : samples_a) {
    for (const auto & sample_b : samples_b) {
      const double dx = sample_a.x - sample_b.x;
      const double dy = sample_a.y - sample_b.y;
      const double distance_sq = dx * dx + dy * dy;

      if (distance_sq < min_distance_sq) {
        min_distance_sq = distance_sq;
        result.min_distance = std::sqrt(distance_sq);
        result.eta_a = sample_a.accumulated_distance / std::max(object_a.speed, 0.1);
        result.eta_b = sample_b.accumulated_distance / std::max(object_b.speed, 0.1);
        result.eta_gap = std::fabs(result.eta_a - result.eta_b);
      }

      if (distance_sq <= distance_threshold_sq) {
        result.overlaps = true;
      }
    }
  }

  return result;
}

// 매칭된 lanelet ID를 미리 지정한 접근로 그룹과 비교해 approach ID를 반환한다.
std::string IntersectionRiskAnalyzerNode::classifyApproach(
  const PredictedObject & object,
  const lanelet::ConstLanelet & matched_lanelet) const
{
  (void)object;
  const auto lanelet_id = matched_lanelet.id();

  for (const auto & [approach_id, lanelet_ids] : approach_lanelet_id_map_) {
    if (std::find(lanelet_ids.begin(), lanelet_ids.end(), lanelet_id) != lanelet_ids.end()) {
      return approach_id;
    }
  }

  return "unknown";
}

std::string IntersectionRiskAnalyzerNode::getApproachLabel(const std::string & approach_id) const
{
  const auto it = approach_label_map_.find(approach_id);
  if (it != approach_label_map_.end()) {
    return it->second;
  }
  return approach_id;
}

IntersectionRiskAnalyzerNode::ManeuverPriority IntersectionRiskAnalyzerNode::getManeuverPriority(
  const std::string & maneuver) const
{
  if (maneuver == "right" || maneuver == "right_turn") {
    return ManeuverPriority::RIGHT_TURN;
  }
  if (maneuver == "straight") {
    return ManeuverPriority::STRAIGHT;
  }
  if (maneuver == "left" || maneuver == "left_turn") {
    return ManeuverPriority::LEFT_TURN;
  }
  return ManeuverPriority::UNKNOWN;
}

// 현재 객체 위치가 이미 conflict zone 내부인지 확인한다.
bool IntersectionRiskAnalyzerNode::isInsideConflictZone(
  const PredictedObject & object,
  const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const
{
  const auto & pos = object.kinematics.initial_pose_with_covariance.pose.position;
  lanelet::BasicPoint2d p(pos.x, pos.y);

  for (const auto & zone_lanelet : conflict_zone_lanelets) {
    if (lanelet::geometry::inside(zone_lanelet, p)) {
      return true;
    }
  }

  return false;
}

// 우선권 규칙: 이미 zone 안에 있는 차량, 먼저 overlap에 도달하는 차량,
// 동시 진입 시 접근로 규칙 순서로 판단한다.
std::string IntersectionRiskAnalyzerNode::decidePriorityVehicle(
  const TrackedObjectInfo & object_a,
  const TrackedObjectInfo & object_b,
  const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets,
  const double eta_a,
  const double eta_b) const
{
  const bool a_in_zone = isInsideConflictZone(object_a.object, conflict_zone_lanelets);
  const bool b_in_zone = isInsideConflictZone(object_b.object, conflict_zone_lanelets);

  if (a_in_zone && !b_in_zone) {
    return object_a.object_id;
  }
  if (b_in_zone && !a_in_zone) {
    return object_b.object_id;
  }

  if (std::isfinite(eta_a) && std::isfinite(eta_b) &&
    std::fabs(eta_a - eta_b) > params_.eta_gap_threshold)
  {
    return eta_a <= eta_b ? object_a.object_id : object_b.object_id;
  }

  const auto maneuver_a = getManeuverPriority(object_a.maneuver);
  const auto maneuver_b = getManeuverPriority(object_b.maneuver);
  if (maneuver_a != maneuver_b) {
    return (static_cast<int>(maneuver_a) > static_cast<int>(maneuver_b)) ?
           object_a.object_id : object_b.object_id;
  }

  const auto rule_a = approach_priority_rules_.find(object_a.approach_id);
  if (rule_a != approach_priority_rules_.end() && rule_a->second.right_of == object_b.approach_id) {
    return object_a.object_id;
  }
  const auto rule_b = approach_priority_rules_.find(object_b.approach_id);
  if (rule_b != approach_priority_rules_.end() && rule_b->second.right_of == object_a.approach_id) {
    return object_b.object_id;
  }

  if (std::isfinite(eta_a) && std::isfinite(eta_b)) {
    return eta_a <= eta_b ? object_a.object_id : object_b.object_id;
  }

  return (object_a.eta_to_zone <= object_b.eta_to_zone) ? object_a.object_id : object_b.object_id;
}

// 두 차량의 predicted path가 horizon 안에서 가까워지면 위험 차량 쌍으로 판단한다.
IntersectionRiskAnalyzerNode::PairRiskResult IntersectionRiskAnalyzerNode::evaluatePairRisk(
  const TrackedObjectInfo & object_a,
  const TrackedObjectInfo & object_b,
  const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const
{
  PairRiskResult result{};
  result.risky = false;
  result.object_id_a = object_a.object_id;
  result.object_id_b = object_b.object_id;
  result.approach_a = object_a.approach_id;
  result.approach_b = object_b.approach_id;
  result.eta_a = std::numeric_limits<double>::infinity();
  result.eta_b = std::numeric_limits<double>::infinity();
  result.eta_gap = std::numeric_limits<double>::infinity();

  const auto overlap = evaluatePathOverlap(object_a, object_b);
  if (!overlap.overlaps) {
    return result;
  }

  result.eta_a = overlap.eta_a;
  result.eta_b = overlap.eta_b;
  result.eta_gap = overlap.eta_gap;
  {
    std::ostringstream ss;
    ss << "path_overlap:min_distance=" << std::fixed << std::setprecision(2)
       << overlap.min_distance << "m";
    result.conflict_zone_id = ss.str();
  }

  result.priority_object_id =
    decidePriorityVehicle(object_a, object_b, conflict_zone_lanelets, result.eta_a, result.eta_b);
  result.non_priority_object_id =
    result.priority_object_id == object_a.object_id ? object_b.object_id : object_a.object_id;

  if (result.priority_object_id == object_a.object_id) {
    result.priority_approach_id = object_a.approach_id;
    result.non_priority_approach_id = object_b.approach_id;
  } else {
    result.priority_approach_id = object_b.approach_id;
    result.non_priority_approach_id = object_a.approach_id;
  }

  result.risky = true;
  return result;
}

bool IntersectionRiskAnalyzerNode::applyLockedPriority(
  PairRiskResult & pair_result,
  const std::string & locked_priority_object_id) const
{
  if (locked_priority_object_id == pair_result.object_id_a) {
    pair_result.priority_object_id = pair_result.object_id_a;
    pair_result.non_priority_object_id = pair_result.object_id_b;
    pair_result.priority_approach_id = pair_result.approach_a;
    pair_result.non_priority_approach_id = pair_result.approach_b;
    return true;
  }

  if (locked_priority_object_id == pair_result.object_id_b) {
    pair_result.priority_object_id = pair_result.object_id_b;
    pair_result.non_priority_object_id = pair_result.object_id_a;
    pair_result.priority_approach_id = pair_result.approach_b;
    pair_result.non_priority_approach_id = pair_result.approach_a;
    return true;
  }

  return false;
}

void IntersectionRiskAnalyzerNode::updatePriorityLock(
  PairRiskResult & pair_result,
  PairHistory & history) const
{
  if (!params_.use_priority_lock) {
    return;
  }

  if (history.has_priority_lock) {
    if (applyLockedPriority(pair_result, history.locked_priority_object_id)) {
      return;
    }

    history.has_priority_lock = false;
    history.locked_priority_object_id.clear();
  }

  history.has_priority_lock = true;
  history.locked_priority_object_id = pair_result.priority_object_id;
}

bool IntersectionRiskAnalyzerNode::isApproachPairAllowed(
  const std::string & approach_id_a, const std::string & approach_id_b) const
{
  if (params_.allowed_conflicting_approach_pairs.empty()) {
    return true;
  }

  const std::string canonical_pair =
    (approach_id_a < approach_id_b) ? (approach_id_a + ":" + approach_id_b) :
    (approach_id_b + ":" + approach_id_a);

  return std::find(
    params_.allowed_conflicting_approach_pairs.begin(),
    params_.allowed_conflicting_approach_pairs.end(),
    canonical_pair) != params_.allowed_conflicting_approach_pairs.end();
}

std::string IntersectionRiskAnalyzerNode::makePairKey(
  const std::string & object_id_a, const std::string & object_id_b) const
{
  if (object_id_a < object_id_b) {
    return object_id_a + "__" + object_id_b;
  }
  return object_id_b + "__" + object_id_a;
}

std::string IntersectionRiskAnalyzerNode::buildConflictZoneId(
  const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const
{
  if (conflict_zone_lanelets.empty()) {
    return "none";
  }

  std::ostringstream ss;
  ss << "lanelets:";
  for (size_t i = 0; i < conflict_zone_lanelets.size(); ++i) {
    ss << conflict_zone_lanelets[i].id();
    if (i + 1 < conflict_zone_lanelets.size()) {
      ss << ",";
    }
  }
  return ss.str();
}

// 파라미터로 지정된 lanelet ID를 conflict zone lanelet 목록으로 변환한다.
std::vector<lanelet::ConstLanelet> IntersectionRiskAnalyzerNode::getConflictZoneLanelets() const
{
  std::vector<lanelet::ConstLanelet> zone_lanelets;

  if (!lanelet_map_ptr_) {
    return zone_lanelets;
  }

  for (const auto lanelet_id : params_.conflict_zone_lanelet_ids) {
    auto it = lanelet_map_ptr_->laneletLayer.find(lanelet_id);
    if (it != lanelet_map_ptr_->laneletLayer.end()) {
      zone_lanelets.push_back(*it);
    }
  }

  return zone_lanelets;
}

// 교차로 전체 위험 상태 요약을 publish한다.
void IntersectionRiskAnalyzerNode::publishStatus(
  const std_msgs::msg::Header & header,
  size_t tracked_objects_count,
  size_t conflicting_pairs_count,
  double min_eta_gap,
  double processing_time_ms)
{
  IntersectionRiskStatus msg;
  msg.header = header;
  msg.has_risk = conflicting_pairs_count > 0;
  msg.tracked_objects_count = static_cast<uint32_t>(tracked_objects_count);
  msg.conflicting_pairs_count = static_cast<uint32_t>(conflicting_pairs_count);
  msg.min_eta_gap = min_eta_gap;
  msg.eta_gap_threshold = params_.eta_gap_threshold;
  msg.processing_time_ms = processing_time_ms;
  pub_status_->publish(msg);
}

// 접근로별 WARNING / STOP 상태를 publish한다.
void IntersectionRiskAnalyzerNode::publishApproachWarnings(
  const std_msgs::msg::Header & header,
  const std::vector<PairRiskResult> & risky_pairs)
{
  ApproachWarningArray msg;
  msg.header = header;

  auto signal_priority = [](const std::string & state) {
      if (state == "STOP") {return 2;}
      if (state == "WARNING") {return 1;}
      return 0;
    };

  auto upsert_warning = [&](const std::string & approach_id, const std::string & signal_state,
      const std::string & reason, const double eta_gap,
      std::unordered_map<std::string, ApproachWarning> & warning_map) {
      auto it = warning_map.find(approach_id);
      if (
        it == warning_map.end() ||
        signal_priority(signal_state) > signal_priority(it->second.signal_state))
      {
        ApproachWarning warning;
        warning.header = header;
        warning.approach_id = getApproachLabel(approach_id);
        warning.signal_state = signal_state;
        warning.reason = reason;
        warning.eta_gap = eta_gap;
        warning_map[approach_id] = warning;
      }
    };

  std::unordered_map<std::string, ApproachWarning> warning_map;

  for (const auto & approach_id : params_.approach_ids) {
    ApproachWarning warning;
    warning.header = header;
    warning.approach_id = getApproachLabel(approach_id);
    warning.signal_state = "SAFE";
    warning.reason = "ETA_SAFE";
    warning.eta_gap = -1.0;
    warning_map[approach_id] = warning;
  }

  for (const auto & pair : risky_pairs) {
    upsert_warning(
      pair.priority_approach_id, "WARNING", "PRIORITY_WARNING", pair.eta_gap, warning_map);

    upsert_warning(
      pair.non_priority_approach_id, "STOP", "PRIORITY_STOP", pair.eta_gap, warning_map);
  }

  const auto now = this->now();
  const auto hold_duration =
    rclcpp::Duration::from_seconds(std::max(params_.warning_hold_duration_sec, 0.0));

  for (auto & [approach_id, warning] : warning_map) {
    if (signal_priority(warning.signal_state) > 0) {
      if (params_.warning_hold_duration_sec > 0.0) {
        held_approach_warnings_[approach_id] = HeldApproachWarning{warning, now + hold_duration};
      } else {
        held_approach_warnings_.erase(approach_id);
      }
      continue;
    }

    auto held_it = held_approach_warnings_.find(approach_id);
    if (held_it == held_approach_warnings_.end()) {
      continue;
    }

    if (now < held_it->second.expires_at) {
      warning = held_it->second.warning;
      warning.header = header;
    } else {
      held_approach_warnings_.erase(held_it);
    }
  }

  for (const auto & approach_id : params_.approach_ids) {
    auto it = warning_map.find(approach_id);
    if (it != warning_map.end()) {
      msg.warnings.push_back(it->second);
    }
  }

  pub_approach_warnings_->publish(msg);
}

// 위험 차량 쌍 상세 목록을 publish한다.
void IntersectionRiskAnalyzerNode::publishConflictingPairs(
  const std_msgs::msg::Header & header,
  const std::vector<PairRiskResult> & risky_pairs)
{
  ConflictingPairArray msg;
  msg.header = header;

  for (const auto & pair : risky_pairs) {
    ConflictingPair pair_msg;
    pair_msg.header = header;
    pair_msg.object_id_a = pair.object_id_a;
    pair_msg.object_id_b = pair.object_id_b;
    pair_msg.approach_a = pair.approach_a;
    pair_msg.approach_b = pair.approach_b;
    pair_msg.priority_object_id = pair.priority_object_id;
    pair_msg.non_priority_object_id = pair.non_priority_object_id;
    pair_msg.conflict_zone_id = pair.conflict_zone_id;
    pair_msg.eta_a = pair.eta_a;
    pair_msg.eta_b = pair.eta_b;
    pair_msg.eta_gap = pair.eta_gap;
    pair_msg.risky = pair.risky;
    msg.pairs.push_back(pair_msg);
  }

  pub_conflicting_pairs_->publish(msg);
}

// RViz 시각화를 위한 최소 디버그 마커를 생성한다.
visualization_msgs::msg::MarkerArray IntersectionRiskAnalyzerNode::createDebugMarkers(
  const std_msgs::msg::Header & header,
  const std::vector<TrackedObjectInfo> & tracked_objects,
  const std::vector<PairRiskResult> & risky_pairs) const
{
  visualization_msgs::msg::MarkerArray marker_array;
  int marker_id = 0;

  for (const auto & tracked_object : tracked_objects) {
    const bool is_priority = std::any_of(
      risky_pairs.begin(), risky_pairs.end(),
      [&](const auto & pair) {return pair.priority_object_id == tracked_object.object_id;});
    const bool is_stop_target = std::any_of(
      risky_pairs.begin(), risky_pairs.end(),
      [&](const auto & pair) {return pair.non_priority_object_id == tracked_object.object_id;});

    if (!is_priority && !is_stop_target) {
      continue;
    }

    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "risk_objects";
    marker.id = marker_id++;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = tracked_object.object.kinematics.initial_pose_with_covariance.pose;
    marker.scale.x = 1.5;
    marker.scale.y = 1.5;
    marker.scale.z = 1.5;
    marker.color.a = 0.8;

    if (is_stop_target) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }

    marker_array.markers.push_back(marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header = header;
    text_marker.ns = "risk_labels";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose = tracked_object.object.kinematics.initial_pose_with_covariance.pose;
    text_marker.pose.position.z += 2.0;
    text_marker.scale.z = 1.0;
    text_marker.color.a = 1.0;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.text = is_stop_target ? "STOP" : "WARNING";
    marker_array.markers.push_back(text_marker);
  }

  return marker_array;
}

}  // namespace crossline_collision_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crossline_collision_detector::IntersectionRiskAnalyzerNode)
