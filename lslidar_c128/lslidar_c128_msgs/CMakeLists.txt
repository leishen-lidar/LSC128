cmake_minimum_required(VERSION 2.8.3)
project(lslidar_c128_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  LslidarC128Layer.msg
  LslidarC128Packet.msg
  LslidarC128Point.msg
  LslidarC128Scan.msg
  LslidarC128Sweep.msg
  LslidarC128ScanUnified.msg
)
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs
)
