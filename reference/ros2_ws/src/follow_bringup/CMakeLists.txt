cmake_minimum_required(VERSION 3.8)
project(follow_bringup)
find_package(ament_cmake REQUIRED)

# 这就是"为什么 ros2 launch 找不到我的文件"的答案：
# 源码目录里的 launch/ 和 config/ 不会自动进 install/，必须显式 install。
# 你现在的仓库用 docker cp 手工搬文件，本质上就是在人肉执行这一段。
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
