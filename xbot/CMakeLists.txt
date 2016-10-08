cmake_minimum_required(VERSION 2.8.3)
project(xbot)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  sensor_msgs
)
catkin_package(
  INCLUDE_DIRS driver
  LIBRARIES xbot robot_cmd robot_frame robot_serial
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  driver
)
add_subdirectory(driver)
aux_source_directory(./src DIR_SRCS)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
add_executable(chai_xbot ${DIR_SRCS})

target_link_libraries(chai_xbot ${catkin_LIBRARIES} driver)

