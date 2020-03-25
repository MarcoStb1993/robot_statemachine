cmake_minimum_required(VERSION 2.8.3)
project(rsm_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  geometry_msgs
)

add_message_files(
   FILES
   Waypoint.msg
   WaypointArray.msg
   OperationMode.msg
   GoalStatus.msg
)

add_service_files(
  FILES
  AddWaypoint.srv
  GetWaypoints.srv
  MoveWaypoint.srv
  RemoveWaypoint.srv
  WaypointVisited.srv
  WaypointUnreachable.srv
  SetWaypointFollowingMode.srv
  SetWaypointRoutine.srv
  GetWaypointRoutines.srv
  SetNavigationGoal.srv
  GetNavigationGoal.srv
  GoalCompleted.srv
  GetRobotPose.srv
  SetOperationMode.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


