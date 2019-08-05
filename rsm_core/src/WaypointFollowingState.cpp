#include <rsm_core/WaypointFollowingState.h>

namespace rsm {

WaypointFollowingState::WaypointFollowingState() {
}

WaypointFollowingState::~WaypointFollowingState() {
}

void WaypointFollowingState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rsm");
	_get_waypoints_service = nh.serviceClient<rsm_msgs::GetWaypoints>(
			"getWaypoints");
	_set_waypoint_following_mode_service = nh.serviceClient<
			rsm_msgs::SetWaypointFollowingMode>(
			"setWaypointFollowingMode");
	_reset_waypoints_service = nh.serviceClient<std_srvs::Trigger>(
			"resetWaypoints");
	_waypoint_visited_service = nh.serviceClient<
			rsm_msgs::WaypointVisited>("waypointVisited");
	_set_navigation_goal_service = nh.serviceClient<
			rsm_msgs::SetNavigationGoal>("setNavigationGoal");
	//initialize variables
	_name = "Waypoint Following";
	_next_waypoint_position = -1;
}

void WaypointFollowingState::onEntry() {
	getWaypoints();
}

void WaypointFollowingState::onActive() {
	int next_reachable_unvisited_waypoint_position = -1;
	if (_waypoint_array.reverse) {
		for (int i = _waypoint_array.waypoints_size - 1; i >= 0; i--) {
			if (!_waypoint_array.waypoints[i].visited
					&& !_waypoint_array.waypoints[i].unreachable) {
				next_reachable_unvisited_waypoint_position = i;
				break;
			}
		}
	} else {
		for (int i = 0; i < _waypoint_array.waypoints_size; i++) {
			if (!_waypoint_array.waypoints[i].visited
					&& !_waypoint_array.waypoints[i].unreachable) {
				next_reachable_unvisited_waypoint_position = i;
				break;
			}
		}
	}
	if (next_reachable_unvisited_waypoint_position != -1) {
		_next_waypoint_position = next_reachable_unvisited_waypoint_position;
		if (!_interrupt_occured) {
			_stateinterface->transitionToVolatileState(
					_stateinterface->getPluginState(
					NAVIGATION_STATE));
		}
	} else {
		switch (_waypoint_array.mode) {
		case 0:		//SINGLE
		{
			abortWaypointFollowing();
			break;
		}
		case 1:		//ROUNDTRIP
		{
			resetWaypoints();
			getWaypoints();
			break;
		}
		case 2:		//PATROL
		{
			rsm_msgs::SetWaypointFollowingMode srv2;
			srv2.request.mode = _waypoint_array.mode;
			srv2.request.reverse = !_waypoint_array.reverse;
			if (!_set_waypoint_following_mode_service.call(srv2)) {
				ROS_ERROR("Failed to call Set Waypoint Following Mode service");
				abortWaypointFollowing();
			}
			resetWaypoints();
			setCurrentWaypointVisited();
			getWaypoints();
			break;
		}
		default:
			break;
		}
	}
}

void WaypointFollowingState::onExit() {
	//Set Navigation goal
	if (_next_waypoint_position >= 0) {
		rsm_msgs::SetNavigationGoal srv;
		srv.request.goal =
				_waypoint_array.waypoints[_next_waypoint_position].pose;
		srv.request.navigationMode = WAYPOINT_FOLLOWING;
		srv.request.waypointPosition = _next_waypoint_position;
		srv.request.routine =
				_waypoint_array.waypoints[_next_waypoint_position].routine;
		if (_set_navigation_goal_service.call(srv)) {
		} else {
			ROS_ERROR("Failed to call Set Navigation Goal service");
		}
	}
}

void WaypointFollowingState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following running";
}

void WaypointFollowingState::onExplorationStop(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following running";
}

void WaypointFollowingState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following running";
}

void WaypointFollowingState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = true;
	message = "Waypoint following stopped";
	abortWaypointFollowing();
}

void WaypointFollowingState::onInterrupt(int interrupt) {
	switch (interrupt) {
	case EMERGENCY_STOP_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
		_interrupt_occured = true;
		break;
	case TELEOPERATION_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
		_interrupt_occured = true;
		break;
	case SIMPLE_GOAL_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(NAVIGATION_STATE));
		_interrupt_occured = true;
		break;
	}
}

void WaypointFollowingState::abortWaypointFollowing() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

void WaypointFollowingState::getWaypoints() {
	rsm_msgs::GetWaypoints srv;
	if (_get_waypoints_service.call(srv)) {
		_waypoint_array = srv.response.waypointArray;
		if (!_waypoint_array.waypoints_size) {
			ROS_ERROR(
					"Waypoint Following stopped because no waypoints are available");
			abortWaypointFollowing();
		}
	} else {
		ROS_ERROR("Failed to call Get Failed Goals service");
		abortWaypointFollowing();
	}
}

void WaypointFollowingState::setCurrentWaypointVisited() {
	//Set first/last waypoint to visited, so the routine won't be called twice
	rsm_msgs::WaypointVisited srv;
	if (_waypoint_array.reverse) {
		srv.request.position = 0;
	} else {
		srv.request.position = _waypoint_array.waypoints_size - 1;
	}
	if (!_waypoint_visited_service.call(srv)) {
		ROS_ERROR("Failed to call Waypoint Visited service");
	}
}

void WaypointFollowingState::resetWaypoints() {
	std_srvs::Trigger srv;
	if (!_reset_waypoints_service.call(srv)) {
		ROS_ERROR("Failed to call Reset Waypoints service");
		abortWaypointFollowing();
	}
}

}
