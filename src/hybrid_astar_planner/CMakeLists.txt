cmake_minimum_required(VERSION 3.0.2)
project(hybrid_astar_planner)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  costmap_2d
  pluginlib
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  angles
)

# NOTE: OMPL (Dubins/Reeds-Shepp heuristics) is declared as a dependency in
# package.xml and will be wired into this CMakeLists in Phase B, when the
# Hybrid A* core that uses it is ported. Phase A (plugin skeleton) does not
# need it, keeping the initial build minimal.

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    nav_core
    costmap_2d
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
)

add_library(${PROJECT_NAME}
  src/HybridAstarPlanner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
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
