cmake_minimum_required(VERSION 3.8)
project(planners)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(common REQUIRED)
find_package(kinematics REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_msgs_stamped REQUIRED)


# Add include directories
include_directories(include/common
                    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(circle_planner src/planner.cpp src/circle_planner.cpp)
ament_target_dependencies(circle_planner rclcpp common kinematics tf2 tf2_ros geometry_msgs sensor_msgs std_msgs std_srvs std_msgs_stamped)
target_include_directories(circle_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(line_planner src/planner.cpp src/line_planner.cpp)
ament_target_dependencies(line_planner rclcpp common kinematics tf2 tf2_ros geometry_msgs sensor_msgs std_msgs std_srvs std_msgs_stamped)
target_include_directories(line_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(sin_planner src/planner.cpp src/sin_planner.cpp)
ament_target_dependencies(sin_planner rclcpp common kinematics tf2 tf2_ros geometry_msgs sensor_msgs std_srvs std_msgs std_msgs_stamped)
target_include_directories(sin_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(rectangle_planner src/planner.cpp src/rectangle_planner.cpp)
ament_target_dependencies(rectangle_planner rclcpp common kinematics tf2 tf2_ros geometry_msgs sensor_msgs std_srvs std_msgs std_msgs_stamped)
target_include_directories(rectangle_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(zero_planner src/planner.cpp src/zero_planner.cpp)
ament_target_dependencies(zero_planner rclcpp common kinematics tf2 tf2_ros geometry_msgs sensor_msgs std_srvs std_msgs std_msgs_stamped)
target_include_directories(zero_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)
  
add_executable(blind_line src/blind_planner.cpp src/blind_line.cpp)
ament_target_dependencies(blind_line common rclcpp geometry_msgs sensor_msgs std_srvs std_msgs_stamped)
target_include_directories(blind_line PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(blind_sine src/blind_planner.cpp src/blind_sine.cpp)
ament_target_dependencies(blind_sine common rclcpp geometry_msgs sensor_msgs std_srvs std_msgs_stamped)
target_include_directories(blind_sine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)


add_executable(blind_circle src/blind_planner.cpp src/blind_circle.cpp)
ament_target_dependencies(blind_circle common rclcpp geometry_msgs sensor_msgs std_srvs std_msgs_stamped) 
target_include_directories(blind_circle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/planners>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(ref_pos_repeater src/ref_pos_repeater.cpp)
ament_target_dependencies(ref_pos_repeater rclcpp common geometry_msgs px4_msgs Eigen3 std_msgs_stamped)
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
  blind_line
  blind_sine
  blind_circle
  rectangle_planner
  zero_planner
  sin_planner
  circle_planner
  line_planner
  ref_pos_repeater
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
