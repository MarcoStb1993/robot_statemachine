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
	_start_stop_cmd_vel_recording_service = nh.serviceClient<std_srvs::SetBool>(
			"startStopCmdVelRecording");

	_waypoint_following = false;
	_goal_active = false;

	_cmd_vel_recording = false;

	_idle_timer = nh.createTimer(ros::Duration(10.0),
			&NavigationState::timerCallback, this, true); //15 seconds idle timer
	_idle_timer.stop();
	_comparison_counter = 0;
	_request_reverse_path_usage_service = nh.serviceClient<std_srvs::Trigger>(
			"requestReversePathUsage");
	_reset_reverse_path_usage_service = nh.serviceClient<std_srvs::Trigger>(
			"resetReversePathUsage");
}

void NavigationState::onEntry() {
	ROS_INFO("NavigationState entered");
	statemachine_msgs::GetNavigationGoal srv;
	if (_get_navigation_goal_service.call(srv)) {
		_nav_goal = srv.response.goal;
		_failed_goals = srv.response.failedGoals.poses;
		_waypoint_following = srv.response.waypointFollowing;
		_waypoint_position = srv.response.waypointPosition;
		_routine = srv.response.routine;
		if (_waypoint_following) {
			_name = "Navigation: Waypoint Following";
		} else {
			_name = "Navigation: Exploration";
		}
	} else {
		ROS_ERROR("Failed to call Get Navigation Goal service");
		abortNavigation();
	}
	_idle_timer.start();
	std_srvs::SetBool srv2;
	srv2.request.data = true;
	if (_start_stop_cmd_vel_recording_service.call(srv2)) {
		_cmd_vel_recording = true;
	} else {
		ROS_ERROR("Failed to call Cmd Vel Recording service");
	}
}

void NavigationState::onActive() {
	//ROS_INFO("NavigationState active");
	if (_move_base_client.isServerConnected()) {
		if (_goal_active) {
			if (_move_base_client.getState().isDone()) {
				if (_move_base_client.getState().state_
						== actionlib::SimpleClientGoalState::SUCCEEDED) {
					std_srvs::Trigger srv;
					if (!_reset_reverse_path_usage_service.call(srv)) {
						ROS_ERROR(
								"Failed to call Reset Reverse Path Usage service");
					}
					if (!_interrupt_occured) {
						if (_waypoint_following) {
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
						} else {
							std_srvs::Trigger srv;
							if (!_reset_failed_goals_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Reset Failed Goals service");
							}
							_stateinterface->transitionToVolatileState(
									_stateinterface->getPluginState(
											MAPPING_STATE));
						}
					}
				} else {
					if (!_interrupt_occured) {
						if (_waypoint_following) {
							statemachine_msgs::WaypointUnreachable srv;
							srv.request.position = _waypoint_position;
							if (!_waypoint_unreachable_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Waypoint Visited service");
							}
							_stateinterface->transitionToVolatileState(
									boost::make_shared<WaypointFollowingState>());
						} else {
							//ROS_INFO("Leave NavigationState with %lu failed goals",_failed_goals.size());
							statemachine_msgs::AddFailedGoal srv;
							srv.request.failedGoal = _nav_goal;
							if (!_add_failed_goal_service.call(srv)) {
								ROS_ERROR(
										"Failed to call Add Failed Goal service");
							}
							_stateinterface->transitionToVolatileState(
									_stateinterface->getPluginState(
											CALCULATEGOAL_STATE));
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
	std_srvs::SetBool srv;
	srv.request.data = false;
	if (!_start_stop_cmd_vel_recording_service.call(srv)) {
		ROS_ERROR("Failed to call Cmd Vel Recording service");
	}
}

void NavigationState::onExplorationStart(bool &success, std::string &message) {
	ROS_INFO("Exploration Start called in NavigationState");
	if (_waypoint_following) {
		success = false;
		message = "Waypoint following running";
	} else {
		success = false;
		message = "Exploration false";
	}
}

void NavigationState::onExplorationStop(bool &success, std::string &message) {
	ROS_INFO("Exploration Stop called in NavigationState");
	if (_waypoint_following) {
		success = false;
		message = "Waypoint following running";
	} else {
		success = true;
		message = "Exploration stopped";
		abortNavigation();
	}
}

void NavigationState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/pause called in NavigationState");
	success = false;
	if (_waypoint_following) {
		message = "Waypoint following running";
	} else {
		message = "Exploration running";
	}
}

void NavigationState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in NavigationState");
	if (_waypoint_following) {
		success = true;
		message = "Waypoint following stopped";
		abortNavigation();
	} else {
		success = false;
		message = "Exploration running";
	}
}

void NavigationState::onInterrupt(int interrupt) {
	if (interrupt == EMERGENCY_STOP_INTERRUPT) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
	} else {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
	}
	_interrupt_occured = true;
}

void NavigationState::timerCallback(const ros::TimerEvent& event) {
	ROS_INFO("Navigation Timeout!");
	std_srvs::Trigger srv;
	if (_request_reverse_path_usage_service.call(srv)) {
		if (srv.response.success) {
			if (!_interrupt_occured) {
				_stateinterface->transitionToVolatileState(
						boost::make_shared<ReversePathState>());
			}
		} else {
			abortNavigation();
		}
	} else {
		ROS_ERROR("Failed to call Request Reverse Path Usage service");
	}
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

void NavigationState::abortNavigation() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(statemachine::NavigationState, statemachine::BaseState)
