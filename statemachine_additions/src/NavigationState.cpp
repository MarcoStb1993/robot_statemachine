#include <statemachine_additions/NavigationState.h>

namespace statemachine {

NavigationState::NavigationState() :
		_move_base_client("move_base", true) {
	ROS_INFO("NavigationState constructed");
	_name = "Navigation";
}

NavigationState::~NavigationState() {
	ROS_INFO("NavigationState destructed");
}

void NavigationState::onSetup() {
	ROS_INFO("NavigationState setup");
	ros::NodeHandle nh("statemachine");
	_get_navigation_goal_service = nh.serviceClient<
			statemachine_msgs::GetNavigationGoal>("getNavigationGoal");
	_add_failed_goal_service =
			nh.serviceClient<statemachine_msgs::AddFailedGoal>("addFailedGoal");
	_reset_failed_goals_service = nh.serviceClient<std_srvs::Trigger>(
			"resetFailedGoals");
	_waypoint_visited_service = nh.serviceClient<
			statemachine_msgs::WaypointVisited>("waypointVisited");
	_waypoint_unreachable_service = nh.serviceClient<
			statemachine_msgs::WaypointUnreachable>("waypointUnreachable");
	_get_robot_pose_service = nh.serviceClient<statemachine_msgs::GetRobotPose>(
			"getRobotPose");

	_navigation_mode = -1;
	_goal_active = false;

	_get_exploration_mode = nh.serviceClient<std_srvs::Trigger>(
			"getExplorationMode");
	_exploration_mode = 1; //finish goals
}

void NavigationState::onEntry() {
	ROS_INFO("NavigationState entered");
	statemachine_msgs::GetNavigationGoal srv;
	if (_get_navigation_goal_service.call(srv)) {
		_nav_goal = srv.response.goal;
		_failed_goals = srv.response.failedGoals.poses;
		_navigation_mode = srv.response.navigationMode;
		_waypoint_position = srv.response.waypointPosition;
		_routine = srv.response.routine;
		switch (_navigation_mode) {
		case EXPLORATION:
			_name = "Navigation: Exploration";
			break;
		case WAYPOINT_FOLLOWING:
			_name = "Navigation: Waypoint Following";
			break;
		case SIMPLE_GOAL:
			_name = "Navigation: Simple Goal";
			break;
		default:
			_name = "Navigation";
			break;
		}
	} else {
		ROS_ERROR("Failed to call Get Navigation Goal service");
		abortNavigation();
	}
	_idle_timer.start();
	if (_navigation_mode == EXPLORATION) {
		std_srvs::Trigger srv2;
		if (_get_exploration_mode.call(srv2)) {
			_exploration_mode = srv2.response.success;
			if (_exploration_mode) {
				_get_goal_obsolete = _nh.subscribe("statemachine/goalObsolete",
						1, &NavigationState::goalObsoleteCallback, this);
			}
		} else {
			ROS_ERROR("Failed to call Get Exploration Mode service");
			abortNavigation();
		}
	}
}

void NavigationState::onActive() {
	//ROS_INFO("NavigationState active");
	if (_move_base_client.isServerConnected()) {
		if (_goal_active) {
			if (_move_base_client.getState().isDone()) {
				if (_move_base_client.getState().state_
						== actionlib::SimpleClientGoalState::SUCCEEDED) {
					if (!_interrupt_occured) {
						switch (_navigation_mode) {
						case EXPLORATION: {
							std_srvs::Trigger srv;
							if (!_reset_failed_goals_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Reset Failed Goals service");
							}
							_stateinterface->transitionToVolatileState(
									_stateinterface->getPluginState(
									MAPPING_STATE));
							break;
						}
						case WAYPOINT_FOLLOWING: {
							statemachine_msgs::WaypointVisited srv;
							srv.request.position = _waypoint_position;
							if (!_waypoint_visited_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Waypoint Visited service");
							}
							if (_routine.empty()) {
								_stateinterface->transitionToVolatileState(
										boost::make_shared<
												WaypointFollowingState>());
							} else {
								_stateinterface->transitionToVolatileState(
										_stateinterface->getPluginState(
										ROUTINE_STATE, _routine));
							}
							break;
						}
						case SIMPLE_GOAL: {
							abortNavigation();
							break;
						}
						default: {
							abortNavigation();
							break;
						}
						}
					}
				} else {
					if (!_interrupt_occured) {
						switch (_navigation_mode) {
						case EXPLORATION: {
							statemachine_msgs::AddFailedGoal srv;
							srv.request.failedGoal = _nav_goal;
							if (!_add_failed_goal_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Add Failed Goal service");
							}
							_stateinterface->transitionToVolatileState(
									_stateinterface->getPluginState(
									CALCULATEGOAL_STATE));
							break;
						}
						case WAYPOINT_FOLLOWING: {
							statemachine_msgs::WaypointUnreachable srv;
							srv.request.position = _waypoint_position;
							if (!_waypoint_unreachable_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Waypoint Visited service");
							}
							_stateinterface->transitionToVolatileState(
									boost::make_shared<WaypointFollowingState>());
							break;
						}
						case SIMPLE_GOAL: {
							abortNavigation();
							break;
						}
						default: {
							abortNavigation();
							break;
						}
						}
					}
				}
			} else {
				comparePose();
			}
		} else {
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose = _nav_goal;
			_move_base_client.sendGoal(goal);
			_goal_active = true;
		}
	}
}

void NavigationState::onExit() {
	ROS_INFO("NavigationState exited");
	if (_goal_active) {
		_move_base_client.cancelGoal();
	}
}

void NavigationState::onExplorationStart(bool &success, std::string &message) {
	switch (_navigation_mode) {
	case EXPLORATION:
		success = false;
		message = "Exploration running";
		break;
	case WAYPOINT_FOLLOWING:
		success = false;
		message = "Waypoint following running";
		break;
	case SIMPLE_GOAL:
		success = false;
		message = "Simple Goal running";
		break;
	default:
		success = false;
		message = "Nothing running";
		break;
	}
}

void NavigationState::onExplorationStop(bool &success, std::string &message) {
	switch (_navigation_mode) {
	case EXPLORATION:
		success = true;
		message = "Exploration stopped";
		abortNavigation();
		break;
	case WAYPOINT_FOLLOWING:
		success = false;
		message = "Waypoint following running";
		break;
	case SIMPLE_GOAL:
		success = false;
		message = "Simple Goal running";
		break;
	default:
		success = false;
		message = "Nothing running";
		break;
	}
}

void NavigationState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	switch (_navigation_mode) {
	case EXPLORATION:
		message = "Exploration running";
		break;
	case WAYPOINT_FOLLOWING:
		message = "Waypoint following running";
		break;
	case SIMPLE_GOAL:
		message = "Simple Goal running";
		break;
	default:
		message = "Nothing running";
		break;
	}
}

void NavigationState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	switch (_navigation_mode) {
	case EXPLORATION:
		success = false;
		message = "Exploration running";
		break;
	case WAYPOINT_FOLLOWING:
		success = true;
		message = "Waypoint following stopped";
		abortNavigation();
		break;
	case SIMPLE_GOAL:
		success = false;
		message = "Simple Goal running";
		break;
	default:
		success = false;
		message = "Nothing running";
		break;
	}
}

void NavigationState::onInterrupt(int interrupt) {
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

void NavigationState::timerCallback(const ros::TimerEvent& event) {
	ROS_INFO("Navigation Timeout!");
	abortNavigation();
}

void NavigationState::comparePose() {
	if (_comparison_counter++ >= 5) { //only compare poses every 5th call to reduce load
		tf::Pose current_pose;
		statemachine_msgs::GetRobotPose srv;
		if (_get_robot_pose_service.call(srv)) {
			tf::poseMsgToTF(srv.response.pose, current_pose);
			tf::Pose pose_difference = current_pose.inverseTimes(_last_pose);
			if (pose_difference.getOrigin().x() < POSE_TOLERANCE
					&& pose_difference.getOrigin().y() < POSE_TOLERANCE
					&& pose_difference.getOrigin().z() < POSE_TOLERANCE
					&& pose_difference.getRotation().x() == 0.0
					&& pose_difference.getRotation().y() == 0.0
					&& pose_difference.getRotation().z() == 0.0
					&& pose_difference.getRotation().w() == 1.0) {
				_idle_timer.start();
			} else {
				_idle_timer.stop();
			}
			_last_pose = current_pose;
			_comparison_counter = 0;
		} else {
			ROS_ERROR("Failed to call Get Robot Pose service");
		}
	}
}

void NavigationState::goalObsoleteCallback(
		const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data && !_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(
				CALCULATEGOAL_STATE));
	}
}

void NavigationState::abortNavigation() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(statemachine::NavigationState, statemachine::BaseState)
