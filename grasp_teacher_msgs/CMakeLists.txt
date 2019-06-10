cmake_minimum_required(VERSION 2.8.3)
project(grasp_teacher_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  genmsg
)

add_service_files(
  DIRECTORY srv
  FILES
  GetPoseStamped.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()
