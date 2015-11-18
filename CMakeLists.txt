cmake_minimum_required(VERSION 2.8.3)
project(trimod_gripper)

find_package(orocos_kdl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf
  tf_conversions
  message_generation
  message_runtime
  pcl_ros
  kdl_parser
  lwr_controllers
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/cmake/Modules/")
set(CMAKE_INCLUDE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include/")


#PCL
find_package(PCL 1.8 REQUIRED  )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#OpenCV
find_package(OpenCV REQUIRED)
if (OPENCV_FOUND)
  set(HAVE_OPENCV ON)
  include_directories(SYSTEM ${OPENCV_INCLUDE_DIRS})
  message("OPENCV INCLUDE DIR:")
  message(${OpenCV_LIBS})
endif(OPENCV_FOUND)

#Aruco
set(ARUCO_LIBS /usr/local/lib/libaruco.so)

MESSAGE("#### LAR TOOLS #####")
find_package(lar_tools)
include_directories(${lar_tools_INCLUDE_DIRS})

MESSAGE("#### LAR VISION #####")
find_package(lar_vision)
include_directories(${lar_vision_INCLUDE_DIRS})


#VisionSystemLegacy
#set(VISION_SYSTEM_LIBS /usr/local/lib/libVisionSystemLegacy.so)

generate_dynamic_reconfigure_options(
  cfg/comau_clusterizer.cfg
  cfg/lwr_manual_photographer.cfg
)


generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

#SOPHUS
#set(SOPHUS_INCLUDE_DIRS /usr/local/include/sophus)
#include_directories(${SOPHUS_INCLUDE_DIRS})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)



## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#    MarkerImageService.srv
# )

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)

include_directories(
  ${catkin_INCLUDE_DIRS}

  ${CMAKE_INCLUDE_PATH}
   ${CMAKE_INCLUDE_PATH}/utils
  ${PCL_INCLUDE_DIRS}
)

MESSAGE(CIAO)
MESSAGE(${CMAKE_INCLUDE_PATH})

# TEST
add_executable(testing  src/test.cpp)
target_link_libraries(testing ${catkin_LIBRARIES} lar_tools )


# UDP TO CONTROLLER
add_executable(lwr_udp_to_controllers  src/lwr_udp_to_controllers.cpp)
target_link_libraries(lwr_udp_to_controllers ${catkin_LIBRARIES} lar_tools)

# MANUAL PHOTOGRAPHER
add_executable(lwr_manual_photographer  src/lwr_manual_photographer.cpp)
target_link_libraries(lwr_manual_photographer ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${PCL_LIBRARIES} lar_tools lar_vision)
add_dependencies(lwr_manual_photographer ${PROJECT_NAME}_gencfg)

# COMAU CLUSTERIZER


add_executable(comau_clusterizer  src/comau_clusterizer.cpp)
target_link_libraries(comau_clusterizer ${OpenCV_LIBS} ${catkin_LIBRARIES}  ${PCL_LIBRARIES} lar_tools lar_vision)
add_dependencies(comau_clusterizer ${PROJECT_NAME}_gencfg)
