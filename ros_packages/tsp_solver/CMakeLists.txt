cmake_minimum_required(VERSION 3.16)
project(tsp_solver)

add_definitions(-DOCTOMAP_NODEBUGOUT)
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
  octomap_msgs
  octomap_ros
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  frontier_detection
  octomap_planner_utils
)


set(LIBRARIES
  MrsTspSolver
)

find_package(catkin REQUIRED
  ${CATKIN_DEPENDENCIES}
)

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

# include Eigen3
find_package(Eigen3 REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

add_service_files(FILES
  SetStart.srv
  Solve.srv
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
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(MrsTspSolver
  src/tsp_solver.cpp
  src/solver_nodelet.cpp
)

add_dependencies(MrsTspSolver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(MrsTspSolver
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

install(DIRECTORY launch config LKH
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
