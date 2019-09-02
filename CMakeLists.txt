cmake_minimum_required(VERSION 2.8.3)
project(ros_leishen_lidar)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  sensor_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_publisher
    src/lidar_publisher.cpp
    src/n301n_lidar.cpp)
add_dependencies(lidar_publisher n301n_lidar_driver_generate_messages_cpp)
 target_link_libraries(lidar_publisher
   ${catkin_LIBRARIES}
 )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS lidar_publisher
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
