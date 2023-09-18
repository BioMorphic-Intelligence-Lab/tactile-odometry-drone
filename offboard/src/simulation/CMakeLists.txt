cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ros_gz REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(common REQUIRED)
find_package(Eigen3 REQUIRED)
#GZ Package import
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-common5 REQUIRED)
find_package(gz-sim7 REQUIRED)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add mocap_publisher plugin
add_library(MocapPub SHARED src/mocap_pub.cpp)
set_property(TARGET MocapPub PROPERTY CXX_STANDARD 17)
target_link_libraries(MocapPub
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
  common::common
)
target_include_directories(MocapPub PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/simulation>
  $<INSTALL_INTERFACE:include>)


add_executable(mocap_forwarder src/mocap_forwarder.cpp)
ament_target_dependencies(mocap_forwarder rclcpp common geometry_msgs px4_msgs Eigen3)
target_include_directories(mocap_forwarder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/simulation>
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
  $<INSTALL_INTERFACE:include>)

add_executable(fake_jointstate_publisher src/fake_jointstate_publisher.cpp)
ament_target_dependencies(fake_jointstate_publisher rclcpp common geometry_msgs sensor_msgs Eigen3)
target_include_directories(fake_jointstate_publisher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/simulation>
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

########### Add Launch Directory ########
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Targets
install(
  TARGETS MocapPub
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS 
  mocap_forwarder
  fake_jointstate_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
