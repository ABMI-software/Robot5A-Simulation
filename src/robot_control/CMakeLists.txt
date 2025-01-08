cmake_minimum_required(VERSION 3.8)
project(robot_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(control_msgs REQUIRED)

# Include directories
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
)

# Add executables
add_executable(moveit_control_simple src/moveit_control_simple.cpp)
add_executable(end_pose_node src/end_pose_node.cpp)
add_executable(aruco_detector_single src/aruco_detector_single.cpp)
add_executable(moveit_control_gui src/moveit_control_gui.cpp)
add_executable(visual_joint_state_publisher src/visual_joint_state_publisher.cpp)
add_executable(aruco_rotation_publisher src/aruco_rotation_publisher.cpp)
add_executable(aruco_detector_double src/aruco_detector_double.cpp)
add_executable(joint_state_bridge src/joint_state_bridge.cpp)
add_executable(arm_state_listener src/arm_state_listener.cpp)  # Added arm_state_listener

# Specify dependencies for each executable
ament_target_dependencies(moveit_control_simple
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(end_pose_node
  rclcpp
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(aruco_detector_single
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  cv_bridge
)

ament_target_dependencies(moveit_control_gui
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
)

ament_target_dependencies(visual_joint_state_publisher
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
)

 ament_target_dependencies(aruco_rotation_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

ament_target_dependencies(aruco_detector_double
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_eigen
  cv_bridge
  OpenCV
  Eigen3
  yaml-cpp
  message_filters
)

ament_target_dependencies(joint_state_bridge
  rclcpp
  sensor_msgs
  control_msgs 
  geometry_msgs
)

ament_target_dependencies(arm_state_listener
  rclcpp
  sensor_msgs
  control_msgs
)

# Link libraries for each executable
target_link_libraries(joint_state_bridge
  yaml-cpp
  Eigen3::Eigen
)

target_link_libraries(arm_state_listener
  yaml-cpp
  Eigen3::Eigen
)

target_link_libraries(aruco_detector_single
  yaml-cpp
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

target_link_libraries(aruco_detector_double
  yaml-cpp
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

target_link_libraries(visual_joint_state_publisher
  yaml-cpp
  Eigen3::Eigen
)

# Link Qt5Widgets for moveit_control_gui
target_link_libraries(moveit_control_gui
  Qt5::Widgets
)

# Install targets
install(TARGETS
  moveit_control_simple
  end_pose_node
  aruco_detector_single
  aruco_rotation_publisher
  moveit_control_gui
  moveit_control_simple
  visual_joint_state_publisher
  aruco_detector_double
  joint_state_bridge
  arm_state_listener  # Added arm_state_listener here
  DESTINATION lib/${PROJECT_NAME}
)

include_directories(
    ${control_msgs_INCLUDE_DIRS}  
    ...
)

# Install launch and config files
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()