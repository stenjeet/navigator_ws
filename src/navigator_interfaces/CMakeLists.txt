cmake_minimum_required(VERSION 3.8)
project(navigator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandmarkObservation.msg"
  "msg/LandmarkObservationArray.msg"
  "msg/Landmark.msg"
  "msg/LandmarkMap.msg"
  DEPENDENCIES std_msgs
)

ament_package()
