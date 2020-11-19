cmake_minimum_required(VERSION 2.8.3)
project(rsm_rqt_plugins)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  rsm_msgs
  std_msgs
  std_srvs
)

set(CMAKE_AUTORCC ON)
find_package(cmake_modules REQUIRED)
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS} include)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS ${INC_DIR}
  LIBRARIES ${PROJECT_NAME}
  ${CMAKE_CURRENT_BINARY_DIR}/..
  ${CMAKE_CURRENT_BINARY_DIR}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp
)

###########
## Build ##
###########

## set variables for Statemachine Control node
set(SRCS_CONTROLS
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

##########################################################################################

include_directories(${INC_DIR} ${catkin_INCLUDE_DIRS})

## Statemachine Control:
add_library(rsm_rqt_plugins ${SRCS_CONTROLS} ${MOCS_SRCS_CONTROLS} ${UI_HEADER_CONTROLS})
target_link_libraries(rsm_rqt_plugins ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
add_dependencies(rsm_rqt_plugins ${catkin_EXPORTED_TARGETS})
  
install(TARGETS rsm_rqt_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
          
install(FILES
  rsm_rqt_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})