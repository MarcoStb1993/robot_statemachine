cmake_minimum_required(VERSION 2.8.3)
project(statemachine_rona_additions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  statemachine
  statemachine_msgs
  rona_msgs
)

catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(statemachine_rona_plugins
    src/RonaNavigationState.cpp)

add_executable(ronaAdditionsServiceProviderNode
	src/AdditionsServiceProviderNode.cpp
	src/AdditionsServiceProvider.cpp)

target_link_libraries(statemachine_rona_plugins
  ${catkin_LIBRARIES}
  )
  target_link_libraries(ronaAdditionsServiceProviderNode
  ${catkin_LIBRARIES}
  )

add_dependencies(statemachine_rona_plugins ${catkin_EXPORTED_TARGETS})
add_dependencies(ronaAdditionsServiceProviderNode ${catkin_EXPORTED_TARGETS})
