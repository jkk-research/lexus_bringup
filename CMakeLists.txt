cmake_minimum_required(VERSION 3.8)
project(lexus_bringup)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pacmod3_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)  # os_pcl_merger_component 

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(ament_dependencies
  rclcpp
  std_msgs
  pacmod3_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
  rclcpp_components
)

# ==== os_pcl_merger_component ====
add_library(os_pcl_merger SHARED src/pcl_merger.cpp) # must be shared!
target_include_directories(os_pcl_merger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(os_pcl_merger ${ament_dependencies})


add_executable(speed_control src/speed_control.cpp)
ament_target_dependencies(speed_control ${ament_dependencies}) 
add_executable(path_steering_and_kmph src/path_steering_and_kmph.cpp)
ament_target_dependencies(path_steering_and_kmph ${ament_dependencies}) 
add_executable(current_pose_from_tf src/current_pose_from_tf.cpp)
ament_target_dependencies(current_pose_from_tf ${ament_dependencies}) 
add_executable(odom src/odom.cpp)
ament_target_dependencies(odom ${ament_dependencies}) 
add_executable(vehicle_status_from_pacmod src/vehicle_status_from_pacmod.cpp)
ament_target_dependencies(vehicle_status_from_pacmod ${ament_dependencies})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# ==== os_pcl_merger_component ====
rclcpp_components_register_node(
  os_pcl_merger
    PLUGIN "merger::OusterPCLMerger"
    EXECUTABLE os_pcl_merger_node
)

install(DIRECTORY 
  launch
  etc
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  speed_control
  path_steering_and_kmph
  current_pose_from_tf
  vehicle_status_from_pacmod
  odom
  DESTINATION lib/${PROJECT_NAME})

  # ==== os_pcl_merger_component ====
ament_export_targets(export_os_pcl_merger)
# do not install either target into lib/${PROJECT_NAME}
install(TARGETS os_pcl_merger
        EXPORT export_os_pcl_merger
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_package()
