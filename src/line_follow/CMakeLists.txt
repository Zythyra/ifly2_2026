cmake_minimum_required(VERSION 3.0.2)
project(line_follow)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
  message_generation
  actionlib
  move_base_msgs
  dynamic_reconfigure
  ucarmain2026
)

find_package(OpenCV REQUIRED COMPONENTS
  core
  highgui
  imgproc
  calib3d
  videoio
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_service_files(
  FILES
  line_follow.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(line_right src/line_right.cpp)
add_executable(line_left src/line_left.cpp)

add_executable(dynamic_awb src/dynamic_awb.cpp)
add_executable(wb src/wb.cpp)

target_link_libraries(line_right
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(line_left
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(dynamic_awb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(wb
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_dependencies(line_right
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(line_left
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

# ============================================================
# 第二套巡线节点：line2_left
# ============================================================

add_executable(line2_left
  src/line2_left.cpp
)

add_dependencies(line2_left
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(line2_left
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# ============================================================
# 第二套右巡线节点：line2_right
# ============================================================

add_executable(line2_right
  src/line2_right.cpp
)

add_dependencies(line2_right
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(line2_right
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# ============================================================
# 省赛红绿灯识别节点：traffic_light
# 识别成功后直接调用 line_follow.srv 巡线服务
# ============================================================

add_executable(traffic_light
  src/traffic_light.cpp
)

add_dependencies(traffic_light
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

# ============================================================
# 国赛红绿灯识别节点：traffic_light2
# 识别成功后直接调用 line_follow.srv 巡线服务
# ============================================================

add_executable(traffic_light2
  src/traffic_light2.cpp
)

add_dependencies(traffic_light2
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(traffic_light2
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)


add_executable(line2o_right
  src/line2o_right.cpp
)

add_dependencies(line2o_right
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(line2o_right
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(line2o_left
  src/line2o_left.cpp
)

add_dependencies(line2o_left
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(line2o_left
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(lineo_right
  src/lineo_right.cpp
)

add_dependencies(lineo_right
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(lineo_right
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)