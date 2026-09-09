cmake_minimum_required(VERSION 3.16)
project(path_planning)

add_definitions(-DOCTOMAP_NODEBUGOUT)
add_definitions(-O3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# disables anoying googletest warning
set(CATKIN_ENABLE_TESTING FALSE)

set(CATKIN_DEPENDENCIES
  cmake_modules
  mrs_lib
  mrs_msgs
  nodelet
  roscpp
  rospy
  octomap_msgs
  octomap_ros
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  frontier_detection
  octomap_planner_utils
)


set(LIBRARIES
  MrsPathPlanning
)

find_package(catkin REQUIRED
  ${CATKIN_DEPENDENCIES}
)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

add_service_files(FILES
  FindSimplifiedPath.srv
)

## Generate added services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS OCTOMAP message_runtime
)

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(MrsPathPlanning
  src/path_planning_nodelet.cpp
  src/path_planner.cpp
)

add_dependencies(MrsPathPlanning
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(MrsPathPlanning
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------


install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
