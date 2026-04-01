cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YawOverride.msg"
  "msg/Waypoint4D.msg"
  "msg/Waypoint4DArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(drone_yaw_360 src/drone_yaw_360.cpp)

ament_target_dependencies(drone_yaw_360
  rclcpp
  nav_msgs
  mavros_msgs
)

target_include_directories(drone_yaw_360 PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
)

rosidl_target_interfaces(drone_yaw_360
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS drone_yaw_360
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(takeoff src/takeoff.cpp)
ament_target_dependencies(takeoff
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
)
install(TARGETS takeoff DESTINATION lib/${PROJECT_NAME})

add_executable(pouso src/pouso.cpp)
ament_target_dependencies(pouso
  rclcpp
  geometry_msgs
  nav_msgs
  mavros_msgs
)
install(TARGETS pouso DESTINATION lib/${PROJECT_NAME})

add_executable(missao_P_T src/missao_P_T.cpp)
ament_target_dependencies(missao_P_T
  rclcpp
)
install(TARGETS missao_P_T DESTINATION lib/${PROJECT_NAME})

add_executable(supervisor_T src/supervisor_T.cpp)
ament_target_dependencies(supervisor_T rclcpp std_msgs)
install(TARGETS supervisor_T DESTINATION lib/${PROJECT_NAME})

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

ament_package()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)