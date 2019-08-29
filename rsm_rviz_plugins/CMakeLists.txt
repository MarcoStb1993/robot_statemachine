cmake_minimum_required(VERSION 2.8.12)
project(rsm_rviz_plugins)

# C++ 11
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rviz
  rsm_msgs
  visualization_msgs
  interactive_markers
  std_msgs
  std_srvs
  pluginlib
  tf
)

#link_directories(${catkin_LIBRARY_DIRS})

# Qt Stuff
if(rviz_QT_VERSION VERSION_LESS "5")
  find_package(Qt4 ${rviz_QT_VERSION} REQUIRED QtCore QtGui)
  include(${QT_USE_FILE})
  macro(qt_wrap_ui)
    qt4_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt4_wrap_cpp(${ARGN})
  endmacro()
else()
  find_package(Qt5 ${rviz_QT_VERSION} REQUIRED Core Widgets)
  set(QT_LIBRARIES Qt5::Widgets)
  macro(qt_wrap_ui)
    qt5_wrap_ui(${ARGN})
  endmacro()
  macro(qt_wrap_cpp)
    qt5_wrap_cpp(${ARGN})
  endmacro()
endif()

set(CMAKE_AUTORCC ON)
find_package(cmake_modules REQUIRED)
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS} include)

catkin_package(
  INCLUDE_DIRS ${INC_DIR}
  LIBRARIES ${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${CMAKE_CURRENT_BINARY_DIR}
  CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## set variables for Statemachine Control node
set(SRCS_CONTROLS
  src/PlantWaypointTool.cpp
  src/RSMControls.cpp
)
set(HDRS_CONTROLS
  src/RSMControls.h
)
set(UIS_CONTROLS
  src/rsm_controls.ui
)

set(INC_DIR
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

##########################################################################################
# qt5_wrap_cpp produces moc files for all headers listed
# qt5_wrap_ui produces .h files for all .ui files listed

## Statemachine Control:
qt5_wrap_cpp(MOCS_SRCS_CONTROLS ${HDRS_CONTROLS})
qt5_wrap_ui(UI_HEADER_CONTROLS ${UIS_CONTROLS})


include_directories(${INC_DIR} ${catkin_INCLUDE_DIRS})

## Statemachine Control:
add_library(${PROJECT_NAME} ${SRCS_CONTROLS} ${MOCS_SRCS_CONTROLS} ${UI_HEADER_CONTROLS})
add_executable(waypointFollowingVisualizationNode
	src/WaypointFollowingVisualizationNode.cpp
	src/WaypointFollowingVisualization.cpp)

target_link_libraries(${PROJECT_NAME} ${rviz_DEFAULT_PLUGIN_LIBRARIES} ${QT_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(waypointFollowingVisualizationNode ${catkin_LIBRARIES})
  
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
add_dependencies(waypointFollowingVisualizationNode ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
  
install(TARGETS waypointFollowingVisualizationNode
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark config files for installation
install(FILES
  rsm_rviz_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
  
install(DIRECTORY media/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/media)


