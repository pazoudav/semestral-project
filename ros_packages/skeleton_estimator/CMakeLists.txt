cmake_minimum_required(VERSION 3.16)
project(skeleton_estimator)

add_definitions(-O3)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# disables anoying googletest warning
set(CATKIN_ENABLE_TESTING FALSE)

# disables anoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

set(CATKIN_DEPENDENCIES
  cmake_modules
  mrs_lib
  nodelet
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  explorer_utils
  octomap_msgs
  message_generation
  message_runtime
)

set(LIBRARIES
  SkeletonEstimator
)

find_package(catkin REQUIRED
  ${CATKIN_DEPENDENCIES}
)

find_package(PCL REQUIRED)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

find_package(yaml-cpp REQUIRED)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

add_message_files(FILES
  Waypoint.msg
  PrecomputedPath.msg
)

## Generate added messages with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  LIBRARIES ${LIBRARIES}
  DEPENDS PCL EIGEN3 OCTOMAP message_runtime
)

include_directories(SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

add_library(SkeletonEstimator
  src/skeleton_estimator_nodelet.cpp
  src/rosa.cpp
  src/extra_del.cpp
  src/precomputed_paths.cpp
)

add_dependencies(SkeletonEstimator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(SkeletonEstimator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  yaml-cpp
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
