cmake_minimum_required(VERSION 3.0.2)
project(roslaunch_sample)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package()

add_subdirectory(src/listner)
add_subdirectory(src/talker)
