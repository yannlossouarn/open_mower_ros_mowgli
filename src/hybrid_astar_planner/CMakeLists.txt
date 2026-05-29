cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  base_local_planner
  pluginlib
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  angles
)

# OMPL provides the Dubins state space used for the constrained heuristic and
# the analytic (Dubins shot) expansion.
find_package(ompl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    nav_core
    costmap_2d
    base_local_planner
    pluginlib
    geometry_msgs
    nav_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
    angles
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/HybridAstarPlanner.cpp
  src/HybridAStar.cpp
  src/CollisionChecker.cpp
  src/Node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
