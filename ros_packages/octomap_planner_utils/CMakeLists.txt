cmake_minimum_required(VERSION 3.16)
project(octomap_planner_utils)

add_definitions(-DOCTOMAP_NODEBUGOUT)
add_definitions(-O3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# disables anoying googletest warning
set(CATKIN_ENABLE_TESTING FALSE)

set(CATKIN_DEPENDENCIES
  cmake_modules
  roscpp
  octomap_msgs
  octomap_ros
  mrs_lib
  mrs_msgs
  )

set(LIBRARIES
  OctomapPlannerUtils
  )

find_package(catkin REQUIRED
  ${CATKIN_DEPENDENCIES}
  )

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS OCTOMAP
  )

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  )

add_library(OctomapPlannerUtils
  src/utils.cpp
  )

add_dependencies(OctomapPlannerUtils
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapPlannerUtils
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
