cmake_minimum_required(VERSION 2.8.3)
project(statemachine_additions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  actionlib
  actionlib_msgs
  pcl_conversions
  pcl_ros
  statemachine
  statemachine_msgs
  visualization_msgs
  rona_msgs
  #exploration_msgs
)

catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(statemachine_plugins
    src/CalculateGoalState.cpp
    src/NavigationState.cpp
    src/RonaNavigationState.cpp
    src/MappingState.cpp
    src/ReversePathState.cpp
    src/ReversingRoutineState.cpp)

add_executable(unboundExplorationNode src/UnboundExplorationNode.cpp)
add_executable(bootUpNode src/BootUpNode.cpp)
add_executable(goalInterceptNode src/GoalInterceptNode.cpp)
add_executable(frontiersMarkerToPointcloudNode src/FrontiersMarkerToPointcloudNode.cpp)

target_link_libraries(statemachine_plugins
  ${catkin_LIBRARIES}
  )
target_link_libraries(unboundExplorationNode
  ${catkin_LIBRARIES}
  )
target_link_libraries(bootUpNode
  ${catkin_LIBRARIES}
  )
target_link_libraries(goalInterceptNode
  ${catkin_LIBRARIES}
  )
  target_link_libraries(frontiersMarkerToPointcloudNode
  ${catkin_LIBRARIES}
  )

add_dependencies(statemachine_plugins ${catkin_EXPORTED_TARGETS})
add_dependencies(unboundExplorationNode ${catkin_EXPORTED_TARGETS})
add_dependencies(bootUpNode ${catkin_EXPORTED_TARGETS})
add_dependencies(goalInterceptNode ${catkin_EXPORTED_TARGETS})
add_dependencies(frontiersMarkerToPointcloudNode ${catkin_EXPORTED_TARGETS})
