cmake_minimum_required(VERSION 3.8)
project(planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)


# Add include directories
include_directories(include/common
                    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(rectangle_planner src/planner.cpp src/rectangle_planner.cpp)
ament_target_dependencies(rectangle_planner rclcpp common geometry_msgs sensor_msgs)
target_include_directories(rectangle_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
  
add_executable(test_trajectory_publisher src/planner.cpp src/test_trajectory.cpp)
ament_target_dependencies(test_trajectory_publisher common rclcpp geometry_msgs sensor_msgs)
target_include_directories(test_trajectory_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(ref_pos_repeater src/ref_pos_repeater.cpp)
ament_target_dependencies(ref_pos_repeater rclcpp common geometry_msgs px4_msgs Eigen3)
target_include_directories(ref_pos_repeater PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)


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

install(TARGETS 
  test_trajectory_publisher
  rectangle_planner
  ref_pos_repeater
  DESTINATION lib/${PROJECT_NAME})

ament_package()
