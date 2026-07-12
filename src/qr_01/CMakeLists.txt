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
  videoio
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
  CATKIN_DEPENDS
    roscpp
    std_msgs
    message_runtime
    cv_bridge
    sensor_msgs
    image_transport
)

# 包含目录
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${ZBar_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 生成可执行文件
add_executable(aruco src/aruco.cpp)
# add_executable(aruco_sub src/aruco_sub.cpp)
add_executable(qr_server_node src/qr_server.cpp)

target_link_libraries(aruco
  ${catkin_LIBRARIES}
  ${ZBar_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(qr_server_node
  ${catkin_LIBRARIES}
  ${ZBar_LIBRARIES}
  ${OpenCV_LIBS}
)
# target_link_libraries(aruco_sub
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBS}
# )


# 加排队锁：等 srv 头文件生成完毕后再编译 C++
add_dependencies(qr_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(aruco
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(qr_server_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)