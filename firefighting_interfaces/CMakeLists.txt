cmake_minimum_required(VERSION 3.8)
project(firefighting_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToWaypoint.srv"     # your existing one
  "srv/StartMission.srv"
  "srv/DeployExtinguisher.srv"
  "srv/RTL.srv"
  "msg/FireDetection.msg"
  "msg/MissionState.msg"
  "msg/TargetPosition.msg"
  DEPENDENCIES 
    std_msgs 
    geometry_msgs
    sensor_msgs
    geographic_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
