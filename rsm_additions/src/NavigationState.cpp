#include <rsm_additions/NavigationState.h>

namespace rsm {

NavigationState::NavigationState() {
}

NavigationState::~NavigationState() {
}

void NavigationState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rsm");
	_get_navigation_goal_service = nh.serviceClient<
			rsm_msgs::GetNavigationGoal>("getNavigationGoal");
	_add_failed_goal_service =
			nh.serviceClient<rsm_msgs::AddFailedGoal>("addFailedGoal");
	_reset_failed_goals_service = nh.serviceClient<std_srvs::Trigger>(
			"resetFailedGoals");
	_waypoint_visited_service = nh.serviceClient<
			rsm_msgs::WaypointVisited>("waypointVisited");
	_waypoint_unreachable_service = nh.serviceClient<
			rsm_msgs::WaypointUnreachable>("waypointUnreachable");
	_get_robot_pose_service = nh.serviceClient<rsm_msgs::GetRobotPose>(
			"getRobotPose");
	_get_reverse_mode_service = nh.serviceClient<std_srvs::Trigger>(
			"getReverseMode");
	_reverse_mode_subscriber = nh.subscribe<std_msgs::Bool>("reverseMode", 10,
			&NavigationState::reverseModeCallback, this);
	_get_exploration_mode_service = nh.serviceClient<std_srvs::Trigger>(
			"getExplorationMode");
	_idle_timer = _nh.createTimer(ros::Duration(15.0),
			&NavigationState::timerCallback, this, true, false);
	//initialize variables
	_name = "Navigation";
	_navigation_mode = -1;
	_goal_active = false;
	_exploration_mode = -1;
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
}

void NavigationState::onEntry() {
	//Request navigation goal from Service Provider
	rsm_msgs::GetNavigationGoal srv;
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
	//Check if reverse mode is active with Service Provider
	std_srvs::Trigger srv2;
	if (_get_reverse_mode_service.call(srv2)) {
		_reverse_mode = srv2.response.success;
		if (_reverse_mode) {
			_move_base_client.reset(
					new MoveBaseClient("move_base_reverse", true));
		} else {
			_move_base_client.reset(new MoveBaseClient("move_base", true));
		}
	} else {
		ROS_ERROR("Failed to call Get Reverse Mode service");
		_move_base_client.reset(new MoveBaseClient("move_base", true));
	}
	//Get exploration mode from Service Provider if exploration is active
	if (_navigation_mode == EXPLORATION) {
		std_srvs::Trigger srv2;
		if (_get_exploration_mode_service.call(srv2)) {
			_exploration_mode = srv2.response.success;
			if (_exploration_mode) {
				_get_goal_obsolete = _nh.subscribe("rsm/goalObsolete",
						1, &NavigationState::goalObsoleteCallback, this);
			}
		} else {
			ROS_ERROR("Failed to call Get Exploration Mode service");
			abortNavigation();
		}
	}
	//Start timer for checking navigation being stuck for too long (15 secs) without proper message from Move Base
	_idle_timer.start();
}

void NavigationState::onActive() {
	if (_move_base_client->isServerConnected()) {
		if (_goal_active) {
			if (_move_base_client->getState().isDone()) {
				if (_move_base_client->getState().state_
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
							rsm_msgs::WaypointVisited srv;
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
							rsm_msgs::AddFailedGoal srv;
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
							rsm_msgs::WaypointUnreachable srv;
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
				//Check if robot moved for navigation failure check
				comparePose();
			}
		} else {
			//Initialize goal
			move_base_msgs::MoveBaseGoal goal;
			goal.target_pose.header.frame_id = "map";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose = _nav_goal;
			_move_base_client->sendGoal(goal);
			_goal_active = true;
		}
	}
}

void NavigationState::onExit() {
	if (_goal_active) {
		_move_base_client->cancelGoal();
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
	case SIMPLE_GOAL_STOP_INTERRUPT:
		if(_navigation_mode==SIMPLE_GOAL){
			_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
		}
		break;
	}
}

void NavigationState::timerCallback(const ros::TimerEvent& event) {
	ROS_ERROR("Navigation aborted because robot appears to be stuck");
	abortNavigation();
}

void NavigationState::comparePose() {
	if (_operation_mode == rsm_msgs::OperationMode::AUTONOMOUS) {
		if (_comparison_counter++ >= 5) { //only compare poses every 5th call to reduce load
			tf::Pose current_pose;
			rsm_msgs::GetRobotPose srv;
			if (_get_robot_pose_service.call(srv)) {
				tf::poseMsgToTF(srv.response.pose, current_pose);
				tf::Pose pose_difference = current_pose.inverseTimes(
						_last_pose);
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
	} else {
		_idle_timer.stop();
	}
}

void NavigationState::goalObsoleteCallback(
		const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data && !_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(
				MAPPING_STATE));
	}
}

void NavigationState::reverseModeCallback(
		const std_msgs::Bool::ConstPtr& reverse_mode) {
	if (_reverse_mode != reverse_mode->data) {
		if (_goal_active) {
			_move_base_client->cancelGoal();
		}
		_goal_active = false;
		_reverse_mode = reverse_mode->data;
		if (_reverse_mode) {
			_move_base_client.reset(
					new MoveBaseClient("move_base_reverse", true));
		} else {
			_move_base_client.reset(new MoveBaseClient("move_base", true));
		}
	}
}

void NavigationState::operationModeCallback(
		const rsm_msgs::OperationMode::ConstPtr& operation_mode) {
	_operation_mode = operation_mode->mode;
}

void NavigationState::abortNavigation() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::NavigationState, rsm::BaseState)
