cmake_minimum_required(VERSION 3.14)
project(crossline_collision_detector)

# ROS Humble interface generation expects the system Python environment.
# A conda-first PATH can make rosidl pick an incompatible interpreter.
set(Python3_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "System Python3 for ROS interface generation" FORCE)
set(PYTHON_EXECUTABLE /usr/bin/python3 CACHE FILEPATH "System Python for ROS interface generation" FORCE)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(autoware_auto_perception_msgs REQUIRED)
find_package(autoware_auto_mapping_msgs REQUIRED)
find_package(tier4_autoware_utils REQUIRED)

find_package(lanelet2_core REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(lanelet2_routing REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/IntersectionRiskStatus.msg"
  "msg/ApproachWarning.msg"
  "msg/ApproachWarningArray.msg"
  "msg/ConflictingPair.msg"
  "msg/ConflictingPairArray.msg"
  "msg/WarningDeviceCommand.msg"
  DEPENDENCIES
    std_msgs
    geometry_msgs
  LIBRARY_NAME ${PROJECT_NAME}
)

ament_auto_find_build_dependencies()

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)

ament_auto_add_library(intersection_risk_analyzer_node SHARED
  src/intersection_risk_analyzer_node.cpp
)

target_link_libraries(intersection_risk_analyzer_node
  ${lanelet2_core_LIBRARIES}
  ${lanelet2_extension_LIBRARIES}
  ${cpp_typesupport_target}
)

rclcpp_components_register_node(intersection_risk_analyzer_node
  PLUGIN "crossline_collision_detector::IntersectionRiskAnalyzerNode"
  EXECUTABLE intersection_risk_analyzer_exe
)

ament_auto_add_library(warning_controller_node SHARED
  src/warning_controller_node.cpp
)

target_link_libraries(warning_controller_node
  ${cpp_typesupport_target}
)

rclcpp_components_register_node(warning_controller_node
  PLUGIN "crossline_collision_detector::WarningControllerNode"
  EXECUTABLE warning_controller_exe
)

ament_auto_add_library(approach_warning_visualizer_node SHARED
  src/approach_warning_visualizer_node.cpp
)

target_link_libraries(approach_warning_visualizer_node
  ${OpenCV_LIBS}
  ${cpp_typesupport_target}
)

rclcpp_components_register_node(approach_warning_visualizer_node
  PLUGIN "crossline_collision_detector::ApproachWarningVisualizerNode"
  EXECUTABLE approach_warning_visualizer_exe
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  USE_SCOPED_HEADER_INSTALL_DIR
)
