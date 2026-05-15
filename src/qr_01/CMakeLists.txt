cmake_minimum_required(VERSION 3.0.2)
project(qr_01)

# 查找 ROS 依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  cv_bridge
  sensor_msgs
  std_msgs
  image_transport
)

# 使用 pkg-config 查找 ZBar
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED zbar)

# 查找 OpenCV 并指定必要组件
find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
)

add_service_files(
  FILES
  qr_srv.srv
  qr_code.srv  #2026年的新的二维码服务
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

# 配置 catkin 包
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES qr_01
  CATKIN_DEPENDS roscpp std_msgs message_runtime cv_bridge sensor_msgs image_transport
)

# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${ZBar_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(aruco src/aruco.cpp)
# add_executable(aruco_sub src/aruco_sub.cpp)
add_executable(qr_server_node src/qr_server.cpp)

# 链接库（规范写法）
target_link_libraries(aruco
  ${catkin_LIBRARIES}
  ${ZBar_LIBRARIES}
  ${OpenCV_LIBS}  # 使用 OpenCV 的变量名 OpenCV_LIBS
)

# 链接 ROS库、OpenCV库 和 ZBar解码库
target_link_libraries(qr_server_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} zbar)

# target_link_libraries(aruco_sub
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )


# 加排队锁：等 srv 头文件生成完毕后再编译 C++
add_dependencies(qr_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})