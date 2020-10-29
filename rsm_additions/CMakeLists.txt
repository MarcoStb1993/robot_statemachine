cmake_minimum_required(VERSION 2.8.3)
project(rsm_additions)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  actionlib
  actionlib_msgs
  rsm_core
  rsm_msgs
  visualization_msgs
  move_base_msgs
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  tf
  tf2
  tf2_ros
)


catkin_package(

)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_library(rsm_plugins
    src/CalculateGoalState.cpp
    src/NavigationState.cpp
    src/MappingDummyState.cpp
    src/RealsenseMappingState.cpp
    src/ReversingRoutineState.cpp)

add_executable(bootUpNode src/BootUpNode.cpp)
add_executable(additionsServiceProviderNode
	src/AdditionsServiceProviderNode.cpp
	src/AdditionsServiceProvider.cpp)
add_executable(gazeboTfNode src/GazeboTfNode.cpp
	src/GazeboToTf.cpp)

target_link_libraries(rsm_plugins
  ${catkin_LIBRARIES}
  )
target_link_libraries(bootUpNode
  ${catkin_LIBRARIES}
  )
  target_link_libraries(additionsServiceProviderNode
  ${catkin_LIBRARIES}
  )
  target_link_libraries(gazeboTfNode
  ${catkin_LIBRARIES}
  )  

add_dependencies(rsm_plugins ${catkin_EXPORTED_TARGETS})
add_dependencies(bootUpNode ${catkin_EXPORTED_TARGETS})
add_dependencies(additionsServiceProviderNode ${catkin_EXPORTED_TARGETS})
add_dependencies(gazeboTfNode ${catkin_EXPORTED_TARGETS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
  
install(TARGETS rsm_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS additionsServiceProviderNode bootUpNode
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
  
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
  
install(FILES
  rsm_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
