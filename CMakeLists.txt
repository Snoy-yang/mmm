cmake_minimum_required(VERSION 3.0.2)
project(bobac3_auto_charge)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ar_track_alvar_msgs
  relative_move
)

catkin_package(
  CATKIN_DEPENDS roscpp ar_track_alvar_msgs relative_move
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(auto_charge_node src/auto_charge_node.cpp)
add_dependencies(auto_charge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(auto_charge_node ${catkin_LIBRARIES})