cmake_minimum_required(VERSION 3.8)
project(follow_msgs)

# 接口包是 CMake 世界，不是 setup.py 世界 —— 这就是阶段 A 里说过的
# "ament_python 不编译，ament_cmake 要编译"的分水岭。
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Track.msg"
  "msg/TrackArray.msg"
  "action/FollowPerson.action"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
