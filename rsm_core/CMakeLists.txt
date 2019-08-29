cmake_minimum_required(VERSION 2.8.3)
project(rsm_core)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pluginlib
  actionlib
  actionlib_msgs
  rsm_msgs
  std_srvs
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rsmNode
	src/RSMNode.cpp
    src/StateInterface.cpp
    src/BaseState.cpp
    src/BootState.cpp
    src/IdleState.cpp
    src/EmergencyStopState.cpp
    src/WaypointFollowingState.cpp
    src/TeleoperationState.cpp)
    
add_executable(serviceProviderNode
	src/ServiceProviderNode.cpp
	src/ServiceProvider.cpp)
	
add_executable(robotControlMuxNode
	src/RobotControlMuxNode.cpp
	src/RobotControlMux.cpp)

target_link_libraries(rsmNode
  ${catkin_LIBRARIES}
  )
target_link_libraries(serviceProviderNode
  ${catkin_LIBRARIES}
  )
target_link_libraries(robotControlMuxNode
  ${catkin_LIBRARIES}
  )
  
add_dependencies(rsmNode ${catkin_EXPORTED_TARGETS})
add_dependencies(serviceProviderNode ${catkin_EXPORTED_TARGETS})
add_dependencies(robotControlMuxNode ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS rsmNode serviceProviderNode robotControlMuxNode
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
