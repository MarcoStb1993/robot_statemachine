cmake_minimum_required(VERSION 2.8.3)
project(statemachine_additions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  actionlib
  actionlib_msgs
  statemachine
  statemachine_msgs
  visualization_msgs
)

catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(statemachine_plugins
    src/CalculateGoalState.cpp
    src/NavigationState.cpp
    src/MappingState.cpp
    src/ReversePathState.cpp
    src/ReversingRoutineState.cpp)

add_executable(bootUpNode src/BootUpNode.cpp)
add_executable(additionsServiceProviderNode
	src/AdditionsServiceProviderNode.cpp
	src/AdditionsServiceProvider.cpp)

target_link_libraries(statemachine_plugins
  ${catkin_LIBRARIES}
  )
target_link_libraries(bootUpNode
  ${catkin_LIBRARIES}
  )
  target_link_libraries(additionsServiceProviderNode
  ${catkin_LIBRARIES}
  )

add_dependencies(statemachine_plugins ${catkin_EXPORTED_TARGETS})
add_dependencies(bootUpNode ${catkin_EXPORTED_TARGETS})
add_dependencies(additionsServiceProviderNode ${catkin_EXPORTED_TARGETS})
