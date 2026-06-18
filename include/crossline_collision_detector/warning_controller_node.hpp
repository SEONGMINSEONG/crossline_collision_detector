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

#ifndef CROSSLINE_COLLISION_DETECTOR__WARNING_CONTROLLER_NODE_HPP_
#define CROSSLINE_COLLISION_DETECTOR__WARNING_CONTROLLER_NODE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include <crossline_collision_detector/msg/approach_warning_array.hpp>
#include <crossline_collision_detector/msg/warning_device_command.hpp>
#include <rclcpp/rclcpp.hpp>

namespace crossline_collision_detector
{

using crossline_collision_detector::msg::ApproachWarningArray;
using crossline_collision_detector::msg::WarningDeviceCommand;

/**
 * @brief 경고 장치 제어 노드
 *
 * 접근로별 경고 상태를 받아 전광판 제어 명령으로 변환한다.
 */
class WarningControllerNode : public rclcpp::Node
{
public:
  explicit WarningControllerNode(const rclcpp::NodeOptions & options);

private:
  std::vector<std::string> approach_ids_;
  std::vector<std::string> board_ids_;
  std::unordered_map<std::string, std::string> approach_to_board_map_;

  rclcpp::Subscription<ApproachWarningArray>::SharedPtr sub_approach_warnings_;
  rclcpp::Publisher<WarningDeviceCommand>::SharedPtr pub_sign_board_cmd_;

  /**
   * @brief 접근로별 경고 상태를 전광판 명령으로 변환
   */
  void onApproachWarnings(const ApproachWarningArray::ConstSharedPtr msg);

  /**
   * @brief 접근로 ID에 매핑된 전광판 ID를 반환
   */
  std::string getBoardIdForApproach(const std::string & approach_id) const;
};

}  // namespace crossline_collision_detector

#endif  // CROSSLINE_COLLISION_DETECTOR__WARNING_CONTROLLER_NODE_HPP_
