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

#include "crossline_collision_detector/warning_controller_node.hpp"

namespace crossline_collision_detector
{

// 장치 제어 노드 초기화: 접근로 경고 입력과 장치 명령 출력 토픽을 연결한다.
WarningControllerNode::WarningControllerNode(const rclcpp::NodeOptions & options)
: Node("warning_controller", options)
{
  using std::placeholders::_1;

  board_ids_ = declare_parameter<std::vector<std::string>>(
    "board_ids", std::vector<std::string>{"board_1", "board_2", "board_3", "board_4"});

  approach_ids_ = declare_parameter<std::vector<std::string>>(
    "approach_ids", std::vector<std::string>{"north", "east", "south", "west"});

  for (size_t i = 0; i < approach_ids_.size(); ++i) {
    const auto & approach_id = approach_ids_[i];
    const auto label_param = "approach_" + approach_id + "_label";
    const auto board_param = "approach_" + approach_id + "_board";
    const auto approach_label = declare_parameter<std::string>(label_param, approach_id);
    const auto default_board_id =
      i < board_ids_.size() ? board_ids_[i] : std::string{};
    approach_to_board_map_[approach_label] =
      declare_parameter<std::string>(board_param, default_board_id);
  }

  sub_approach_warnings_ = create_subscription<ApproachWarningArray>(
    "~/input/approach_warnings", rclcpp::QoS{1},
    std::bind(&WarningControllerNode::onApproachWarnings, this, _1));

  pub_sign_board_cmd_ = create_publisher<WarningDeviceCommand>("~/output/sign_board_cmd", 1);
}

std::string WarningControllerNode::getBoardIdForApproach(const std::string & approach_id) const
{
  const auto it = approach_to_board_map_.find(approach_id);
  if (it != approach_to_board_map_.end()) {
    return it->second;
  }
  return "";
}

// 접근로별 경고 상태를 받아 전광판 제어 명령으로 변환한다.
void WarningControllerNode::onApproachWarnings(const ApproachWarningArray::ConstSharedPtr msg)
{
  std::unordered_map<std::string, WarningDeviceCommand> board_commands;

  for (const auto & board_id : board_ids_) {
    WarningDeviceCommand cmd;
    cmd.header = msg->header;
    cmd.device_id = board_id;
    cmd.signal_state = "SAFE";
    cmd.active = false;
    board_commands[board_id] = cmd;
  }

  for (const auto & warning : msg->warnings) {
    const auto board_id = getBoardIdForApproach(warning.approach_id);
    if (board_id.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "No board mapping configured for approach '%s'",
        warning.approach_id.c_str());
      continue;
    }

    auto it = board_commands.find(board_id);
    if (it == board_commands.end()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Mapped board '%s' is not in board_ids",
        board_id.c_str());
      continue;
    }

    it->second.signal_state = warning.signal_state;
    it->second.active = warning.signal_state != "SAFE";
  }

  for (const auto & [board_id, cmd] : board_commands) {
    (void)board_id;
    pub_sign_board_cmd_->publish(cmd);
  }
}

}  // namespace crossline_collision_detector

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(crossline_collision_detector::WarningControllerNode)
