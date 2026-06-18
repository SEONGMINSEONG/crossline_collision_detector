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

#ifndef CROSSLINE_COLLISION_DETECTOR__INTERSECTION_RISK_ANALYZER_NODE_HPP_
#define CROSSLINE_COLLISION_DETECTOR__INTERSECTION_RISK_ANALYZER_NODE_HPP_

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_object.hpp>
#include <autoware_auto_perception_msgs/msg/predicted_objects.hpp>
#include <crossline_collision_detector/msg/approach_warning_array.hpp>
#include <crossline_collision_detector/msg/conflicting_pair_array.hpp>
#include <crossline_collision_detector/msg/intersection_risk_status.hpp>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace crossline_collision_detector
{

using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObject;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using crossline_collision_detector::msg::ApproachWarning;
using crossline_collision_detector::msg::ApproachWarningArray;
using crossline_collision_detector::msg::ConflictingPair;
using crossline_collision_detector::msg::ConflictingPairArray;
using crossline_collision_detector::msg::IntersectionRiskStatus;

/**
 * @brief 교차로 충돌 위험 분석 노드
 *
 * LiDAR 기반 PredictedObjects와 Lanelet map을 입력으로 받아
 * conflict zone 기준 ETA 차이를 계산하고 접근로별 WARNING / STOP 상태를 생성한다.
 */
class IntersectionRiskAnalyzerNode : public rclcpp::Node
{
public:
  explicit IntersectionRiskAnalyzerNode(const rclcpp::NodeOptions & options);

private:
  /**
   * @brief 노드 파라미터
   */
  struct Parameters
  {
    double eta_gap_threshold;
    double prediction_horizon_m;
    double path_overlap_distance_threshold;
    double min_vehicle_speed;
    bool use_consecutive_validation;
    int consecutive_count_threshold;
    bool use_priority_lock;
    int priority_unlock_count_threshold;
    double warning_hold_duration_sec;
    std::vector<std::int64_t> conflict_zone_lanelet_ids;
    std::vector<std::string> approach_ids;
    std::vector<std::string> allowed_conflicting_approach_pairs;
    double stable_id_grid_size;
    std::string traffic_rule_location;
    std::string traffic_rule_participant;
  };

  enum class ManeuverPriority
  {
    UNKNOWN = 0,
    LEFT_TURN = 1,
    STRAIGHT = 2,
    RIGHT_TURN = 3
  };

  struct ApproachPriorityRule
  {
    std::string maneuver;
    std::string right_of;
  };

  /**
   * @brief 분석 대상 차량 정보
   */
  struct TrackedObjectInfo
  {
    std::string object_id;
    std::string approach_id;
    std::string maneuver;
    double speed;
    double eta_to_zone;
    bool reaches_conflict_zone;
    PredictedObject object;
  };

  /**
   * @brief 차량 쌍 위험 분석 결과
   */
  struct PairRiskResult
  {
    bool risky;
    std::string object_id_a;
    std::string object_id_b;
    std::string priority_object_id;
    std::string non_priority_object_id;
    std::string priority_approach_id;
    std::string non_priority_approach_id;
    std::string approach_a;
    std::string approach_b;
    std::string conflict_zone_id;
    double eta_a;
    double eta_b;
    double eta_gap;
  };

  struct PathSample
  {
    double x;
    double y;
    double accumulated_distance;
  };

  struct PathOverlapResult
  {
    bool overlaps{false};
    double min_distance{0.0};
    double eta_a{0.0};
    double eta_b{0.0};
    double eta_gap{0.0};
  };

  /**
   * @brief 차량 쌍별 연속 위험 이력
   */
  struct PairHistory
  {
    int consecutive_risk_count{0};
    int consecutive_miss_count{0};
    bool has_priority_lock{false};
    std::string locked_priority_object_id;
  };

  struct HeldApproachWarning
  {
    ApproachWarning warning;
    rclcpp::Time expires_at;
  };

  Parameters params_;

  rclcpp::Subscription<PredictedObjects>::SharedPtr sub_objects_;
  rclcpp::Subscription<HADMapBin>::SharedPtr sub_map_;

  rclcpp::Publisher<IntersectionRiskStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<ApproachWarningArray>::SharedPtr pub_approach_warnings_;
  rclcpp::Publisher<ConflictingPairArray>::SharedPtr pub_conflicting_pairs_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

  std::unordered_map<std::string, std::vector<std::int64_t>> approach_lanelet_id_map_;
  std::unordered_map<std::string, ApproachPriorityRule> approach_priority_rules_;
  std::unordered_map<std::string, std::string> approach_label_map_;
  std::unordered_map<std::string, PairHistory> pair_history_;
  std::unordered_map<std::string, HeldApproachWarning> held_approach_warnings_;

  /**
   * @brief Lanelet map 수신 콜백
   */
  void onMap(const HADMapBin::ConstSharedPtr msg);

  /**
   * @brief 추적 객체 수신 및 위험 분석 수행 콜백
   */
  void onObjects(const PredictedObjects::ConstSharedPtr msg);

  /**
   * @brief 객체 위치 기준으로 포함된 lanelet을 찾음
   */
  boost::optional<lanelet::ConstLanelet> findClosestLanelet(const PredictedObject & object) const;

  /**
   * @brief 주어진 lanelet ID가 conflict zone에 속하는지 확인
   */
  bool isConflictZoneLaneletId(lanelet::Id lanelet_id) const;

  /**
   * @brief 객체 속도를 계산함
   */
  double getObjectSpeed(const PredictedObject & object) const;

  /**
   * @brief 위치 기반 stable object ID를 생성함
   */
  std::string getStableObjectId(const PredictedObject & object) const;
  std::string uuidToString(const unique_identifier_msgs::msg::UUID & uuid) const;

  /**
   * @brief 객체 predicted path가 conflict zone에 도달하는지 확인
   */
  bool reachesConflictZone(
    const PredictedObject & object,
    const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const;

  /**
   * @brief 객체가 conflict zone에 도달할 예상 시간을 계산
   */
  double calculateEtaToConflictZone(
    const PredictedObject & object,
    const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const;

  /**
   * @brief 객체 predicted path를 horizon 거리 안에서 샘플링
   */
  std::vector<PathSample> samplePredictedPathWithinHorizon(const PredictedObject & object) const;

  /**
   * @brief 두 차량 predicted path가 horizon 안에서 가까워지는지 평가
   */
  PathOverlapResult evaluatePathOverlap(
    const TrackedObjectInfo & object_a,
    const TrackedObjectInfo & object_b) const;

  /**
   * @brief 매칭된 lanelet ID를 기준으로 접근로를 판별
   */
  std::string classifyApproach(
    const PredictedObject & object,
    const lanelet::ConstLanelet & matched_lanelet) const;

  /**
   * @brief 내부 접근로 ID를 출력용 이름으로 변환
   */
  std::string getApproachLabel(const std::string & approach_id) const;

  ManeuverPriority getManeuverPriority(const std::string & maneuver) const;

  /**
   * @brief 현재 객체가 이미 conflict zone 내부에 있는지 확인
   */
  bool isInsideConflictZone(
    const PredictedObject & object,
    const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const;

  /**
   * @brief 두 차량 중 우선권 차량을 결정
   */
  std::string decidePriorityVehicle(
    const TrackedObjectInfo & object_a,
    const TrackedObjectInfo & object_b,
    const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets,
    double eta_a,
    double eta_b) const;

  /**
   * @brief 두 차량 쌍의 predicted path overlap 기반 위험 여부를 평가
   */
  PairRiskResult evaluatePairRisk(
    const TrackedObjectInfo & object_a,
    const TrackedObjectInfo & object_b,
    const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const;

  /**
   * @brief 이미 잠긴 우선권 차량을 현재 위험 결과에 반영
   */
  bool applyLockedPriority(
    PairRiskResult & pair_result,
    const std::string & locked_priority_object_id) const;

  /**
   * @brief 위험쌍 이력에 우선권 lock을 적용하거나 새로 저장
   */
  void updatePriorityLock(PairRiskResult & pair_result, PairHistory & history) const;

  /**
   * @brief 두 접근로 조합이 위험 비교 대상인지 확인
   */
  bool isApproachPairAllowed(
    const std::string & approach_id_a, const std::string & approach_id_b) const;

  /**
   * @brief 파라미터로 지정된 conflict zone lanelet 목록을 가져옴
   */
  std::vector<lanelet::ConstLanelet> getConflictZoneLanelets() const;
  std::string buildConflictZoneId(
    const std::vector<lanelet::ConstLanelet> & conflict_zone_lanelets) const;

  /**
   * @brief 두 차량 ID로 pair history key를 생성
   */
  std::string makePairKey(const std::string & object_id_a, const std::string & object_id_b) const;

  /**
   * @brief 교차로 전체 위험 상태를 publish
   */
  void publishStatus(
    const std_msgs::msg::Header & header,
    size_t tracked_objects_count,
    size_t conflicting_pairs_count,
    double min_eta_gap,
    double processing_time_ms);

  /**
   * @brief 접근로별 WARNING / STOP 상태를 publish
   */
  void publishApproachWarnings(
    const std_msgs::msg::Header & header,
    const std::vector<PairRiskResult> & risky_pairs);

  /**
   * @brief 위험 차량 쌍 상세 정보를 publish
   */
  void publishConflictingPairs(
    const std_msgs::msg::Header & header,
    const std::vector<PairRiskResult> & risky_pairs);

  /**
   * @brief RViz 시각화용 디버그 마커 생성
   */
  visualization_msgs::msg::MarkerArray createDebugMarkers(
    const std_msgs::msg::Header & header,
    const std::vector<TrackedObjectInfo> & tracked_objects,
    const std::vector<PairRiskResult> & risky_pairs) const;
};

}  // namespace crossline_collision_detector

#endif  // CROSSLINE_COLLISION_DETECTOR__INTERSECTION_RISK_ANALYZER_NODE_HPP_
