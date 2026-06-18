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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "crossline_collision_detector/msg/approach_warning.hpp"
#include "crossline_collision_detector/msg/approach_warning_array.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace crossline_collision_detector
{

using crossline_collision_detector::msg::ApproachWarning;
using crossline_collision_detector::msg::ApproachWarningArray;

namespace
{

cv::Scalar stateColor(const std::string & state)
{
  if (state == "SAFE") {
    return cv::Scalar(55, 190, 55);
  }
  if (state == "WARNING") {
    return cv::Scalar(0, 205, 255);
  }
  if (state == "STOP") {
    return cv::Scalar(50, 50, 235);
  }
  return cv::Scalar(120, 120, 120);
}

cv::Scalar etaGapColor(const double eta_gap, const double safe_threshold)
{
  if (!std::isfinite(eta_gap) || eta_gap < 0.0) {
    return cv::Scalar(55, 190, 55);
  }

  const double threshold = std::max(safe_threshold, 0.001);
  const double ratio = std::min(std::max(eta_gap / threshold, 0.0), 1.0);
  const auto red = static_cast<int>((1.0 - ratio) * 235.0);
  const auto green = static_cast<int>(ratio * 190.0);
  return cv::Scalar(40, green, red);
}

std::string normalizeState(const std::string & state)
{
  if (state == "SAFE" || state == "WARNING" || state == "STOP") {
    return state;
  }
  return "NO_DATA";
}

std::string formatEtaGap(const double eta_gap)
{
  if (!std::isfinite(eta_gap) || eta_gap < 0.0) {
    return "ETA: safe";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << eta_gap << " s";
  return "ETA: " + stream.str();
}

void putCenteredText(
  cv::Mat & image, const std::string & text, const cv::Point & center, const double scale,
  const cv::Scalar & color, const int thickness = 2)
{
  int baseline = 0;
  const auto size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
  const cv::Point origin(center.x - size.width / 2, center.y + size.height / 2);
  cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA);
}

}  // namespace

class ApproachWarningVisualizerNode : public rclcpp::Node
{
public:
  explicit ApproachWarningVisualizerNode(const rclcpp::NodeOptions & options)
  : Node("approach_warning_visualizer", options)
  {
    window_name_ = declare_parameter<std::string>("window_name", "Approach Warning Dashboard");
    cell_size_ = declare_parameter<int>("cell_size", 240);
    margin_ = declare_parameter<int>("margin", 16);
    refresh_rate_hz_ = declare_parameter<double>("refresh_rate_hz", 20.0);
    eta_gap_green_threshold_ = declare_parameter<double>("eta_gap_green_threshold", 2.5);
    approach_ids_ = declare_parameter<std::vector<std::string>>(
      "approach_ids", std::vector<std::string>{"north", "east", "south", "west"});

    for (const auto & approach_id : approach_ids_) {
      const auto label = declare_parameter<std::string>(
        "approach_" + approach_id + "_label", approach_id);
      approach_labels_[approach_id] = label;
    }

    initializeWarnings();

    sub_approach_warnings_ = create_subscription<ApproachWarningArray>(
      "~/input/approach_warnings", rclcpp::QoS{1},
      std::bind(&ApproachWarningVisualizerNode::onApproachWarnings, this, std::placeholders::_1));

    cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);

    const auto refresh_period = std::chrono::duration<double>(
      1.0 / std::max(refresh_rate_hz_, 1.0));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(refresh_period),
      std::bind(&ApproachWarningVisualizerNode::render, this));
  }

  ~ApproachWarningVisualizerNode() override
  {
    cv::destroyWindow(window_name_);
  }

private:
  struct Tile
  {
    std::string approach_id;
    std::string label;
    std::string display_name;
    std::string short_name;
    int grid_x;
    int grid_y;
  };

  void initializeWarnings()
  {
    for (const auto & tile : buildTiles()) {
      ApproachWarning warning;
      warning.approach_id = tile.label;
      warning.signal_state = "NO_DATA";
      warning.reason = "NO_DATA";
      warning.eta_gap = -1.0;
      warnings_[tile.approach_id] = warning;
    }
  }

  void onApproachWarnings(const ApproachWarningArray::ConstSharedPtr msg)
  {
    has_update_ = true;
    latest_update_time_ = std::chrono::steady_clock::now();

    std::unordered_map<std::string, ApproachWarning> warning_by_key;
    for (const auto & warning : msg->warnings) {
      warning_by_key[warning.approach_id] = warning;
    }

    for (const auto & tile : buildTiles()) {
      const auto label_it = warning_by_key.find(tile.label);
      if (label_it != warning_by_key.end()) {
        warnings_[tile.approach_id] = label_it->second;
        continue;
      }

      const auto id_it = warning_by_key.find(tile.approach_id);
      if (id_it != warning_by_key.end()) {
        warnings_[tile.approach_id] = id_it->second;
        continue;
      }

      ApproachWarning warning;
      warning.approach_id = tile.label;
      warning.signal_state = "NO_DATA";
      warning.reason = "NO_DATA";
      warning.eta_gap = -1.0;
      warnings_[tile.approach_id] = warning;
    }
  }

  std::vector<Tile> buildTiles() const
  {
    std::vector<Tile> tiles;
    tiles.reserve(approach_ids_.size());

    for (const auto & approach_id : approach_ids_) {
      const auto label_it = approach_labels_.find(approach_id);
      const auto label = label_it == approach_labels_.end() ? approach_id : label_it->second;
      tiles.push_back(
        Tile{
          approach_id, label, displayName(approach_id), shortName(approach_id),
          gridX(approach_id), gridY(approach_id)});
    }

    return tiles;
  }

  void render()
  {
    const int width = 3 * cell_size_ + 4 * margin_;
    const int height = 3 * cell_size_ + 4 * margin_;
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(28, 31, 36));

    drawSummaryTile(image, cellRect(1, 1));
    for (const auto & tile : buildTiles()) {
      drawApproachTile(image, tile, cellRect(tile.grid_x, tile.grid_y));
    }

    cv::imshow(window_name_, image);
    cv::waitKey(1);
  }

  cv::Rect cellRect(const int grid_x, const int grid_y) const
  {
    return cv::Rect(
      margin_ + grid_x * (cell_size_ + margin_),
      margin_ + grid_y * (cell_size_ + margin_),
      cell_size_, cell_size_);
  }

  void drawApproachTile(cv::Mat & image, const Tile & tile, const cv::Rect & rect) const
  {
    const auto warning_it = warnings_.find(tile.approach_id);
    const auto warning = warning_it == warnings_.end() ? ApproachWarning{} : warning_it->second;
    const auto state = normalizeState(warning.signal_state);
    const auto color = stateColor(state);

    drawTileBase(image, rect, color);

    putCenteredText(
      image, tile.display_name, cv::Point(rect.x + rect.width / 2, rect.y + 38),
      0.82, cv::Scalar(235, 238, 242), 2);

    const cv::Rect status_rect(
      rect.x + rect.width / 2 - 48, rect.y + 62, 96, 64);
    cv::rectangle(image, status_rect, color, cv::FILLED, cv::LINE_AA);
    cv::rectangle(image, status_rect, cv::Scalar(245, 245, 245), 2, cv::LINE_AA);
    putCenteredText(
      image, tile.short_name, cv::Point(
        status_rect.x + status_rect.width / 2,
        status_rect.y + status_rect.height / 2), 0.82, cv::Scalar(20, 23, 28), 2);

    putCenteredText(
      image, state, cv::Point(rect.x + rect.width / 2, rect.y + 152),
      0.72, color, 2);

    const auto eta_color = etaGapColor(warning.eta_gap, eta_gap_green_threshold_);
    drawEtaBar(image, rect, warning.eta_gap, eta_color);
    putCenteredText(
      image, formatEtaGap(warning.eta_gap), cv::Point(rect.x + rect.width / 2, rect.y + 195),
      0.55, eta_color, 2);

    putCenteredText(
      image, warning.reason.empty() ? "NO_DATA" : warning.reason,
      cv::Point(rect.x + rect.width / 2, rect.y + 222), 0.38, cv::Scalar(170, 178, 188), 1);
  }

  void drawSummaryTile(cv::Mat & image, const cv::Rect & rect) const
  {
    const auto worst_state = worstState();
    const auto min_eta_gap = minEtaGap();
    const auto color = stateColor(worst_state);

    drawTileBase(image, rect, color);

    putCenteredText(
      image, "SUMMARY", cv::Point(rect.x + rect.width / 2, rect.y + 40),
      0.74, cv::Scalar(235, 238, 242), 2);
    putCenteredText(
      image, worst_state, cv::Point(rect.x + rect.width / 2, rect.y + 96),
      0.88, color, 2);

    const auto eta_color = etaGapColor(min_eta_gap, eta_gap_green_threshold_);
    putCenteredText(
      image, minEtaGapText(min_eta_gap), cv::Point(rect.x + rect.width / 2, rect.y + 142),
      0.56, eta_color, 2);

    putCenteredText(
      image, lastUpdateText(), cv::Point(rect.x + rect.width / 2, rect.y + 185),
      0.42, cv::Scalar(185, 191, 200), 1);

    putCenteredText(
      image, "N/E/S/W + ETA", cv::Point(rect.x + rect.width / 2, rect.y + 218),
      0.42, cv::Scalar(150, 158, 168), 1);
  }

  void drawTileBase(cv::Mat & image, const cv::Rect & rect, const cv::Scalar & color) const
  {
    cv::rectangle(image, rect, cv::Scalar(44, 49, 58), cv::FILLED, cv::LINE_AA);
    cv::rectangle(image, rect, color, 4, cv::LINE_AA);
  }

  void drawEtaBar(
    cv::Mat & image,
    const cv::Rect & rect,
    const double eta_gap,
    const cv::Scalar & eta_color) const
  {
    const cv::Rect bar_bg(rect.x + 26, rect.y + 170, rect.width - 52, 10);
    cv::rectangle(image, bar_bg, cv::Scalar(70, 76, 86), cv::FILLED, cv::LINE_AA);

    double ratio = 1.0;
    if (std::isfinite(eta_gap) && eta_gap >= 0.0) {
      ratio = std::min(std::max(eta_gap / std::max(eta_gap_green_threshold_, 0.001), 0.0), 1.0);
    }

    const cv::Rect bar_fill(
      bar_bg.x, bar_bg.y, static_cast<int>(bar_bg.width * ratio), bar_bg.height);
    cv::rectangle(image, bar_fill, eta_color, cv::FILLED, cv::LINE_AA);
  }

  std::string worstState() const
  {
    int priority = 0;
    std::string worst = "NO_DATA";
    for (const auto & [_, warning] : warnings_) {
      const auto state = normalizeState(warning.signal_state);
      const int current_priority = statePriority(state);
      if (current_priority > priority) {
        priority = current_priority;
        worst = state;
      }
    }

    if (priority == 0 && hasAnySafe()) {
      return "SAFE";
    }
    return worst;
  }

  bool hasAnySafe() const
  {
    return std::any_of(
      warnings_.begin(), warnings_.end(), [](const auto & item) {
        return normalizeState(item.second.signal_state) == "SAFE";
      });
  }

  int statePriority(const std::string & state) const
  {
    if (state == "STOP") {
      return 3;
    }
    if (state == "WARNING") {
      return 2;
    }
    if (state == "SAFE") {
      return 1;
    }
    return 0;
  }

  double minEtaGap() const
  {
    double min_eta_gap = std::numeric_limits<double>::infinity();
    for (const auto & [_, warning] : warnings_) {
      if (std::isfinite(warning.eta_gap) && warning.eta_gap >= 0.0) {
        min_eta_gap = std::min(min_eta_gap, warning.eta_gap);
      }
    }
    return min_eta_gap;
  }

  std::string minEtaGapText(const double min_eta_gap) const
  {
    if (!std::isfinite(min_eta_gap)) {
      return "MIN ETA: safe";
    }

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << min_eta_gap << " s";
    return "MIN ETA: " + stream.str();
  }

  std::string lastUpdateText() const
  {
    if (!has_update_) {
      return "WAITING TOPIC";
    }

    const auto age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - latest_update_time_).count();
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(1) << age << " s ago";
    return "UPDATED " + stream.str();
  }

  std::string displayName(const std::string & approach_id) const
  {
    if (approach_id == "north") {
      return "NORTH";
    }
    if (approach_id == "east") {
      return "EAST";
    }
    if (approach_id == "south") {
      return "SOUTH";
    }
    if (approach_id == "west") {
      return "WEST";
    }
    return approach_id;
  }

  std::string shortName(const std::string & approach_id) const
  {
    if (approach_id == "north") {
      return "N";
    }
    if (approach_id == "east") {
      return "E";
    }
    if (approach_id == "south") {
      return "S";
    }
    if (approach_id == "west") {
      return "W";
    }
    return "?";
  }

  int gridX(const std::string & approach_id) const
  {
    if (approach_id == "west") {
      return 0;
    }
    if (approach_id == "east") {
      return 2;
    }
    return 1;
  }

  int gridY(const std::string & approach_id) const
  {
    if (approach_id == "north") {
      return 0;
    }
    if (approach_id == "south") {
      return 2;
    }
    return 1;
  }

  std::string window_name_;
  int cell_size_{240};
  int margin_{16};
  double refresh_rate_hz_{20.0};
  double eta_gap_green_threshold_{2.5};
  std::vector<std::string> approach_ids_;
  std::unordered_map<std::string, std::string> approach_labels_;
  std::unordered_map<std::string, ApproachWarning> warnings_;
  bool has_update_{false};
  std::chrono::steady_clock::time_point latest_update_time_{};

  rclcpp::Subscription<ApproachWarningArray>::SharedPtr sub_approach_warnings_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace crossline_collision_detector

RCLCPP_COMPONENTS_REGISTER_NODE(crossline_collision_detector::ApproachWarningVisualizerNode)
