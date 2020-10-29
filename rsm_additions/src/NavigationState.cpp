#include <rsm_additions/NavigationState.h>

namespace rsm {

NavigationState::NavigationState() {
}

NavigationState::~NavigationState() {
}

void NavigationState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle private_nh("~");
	private_nh.param("pose_tolerance", _pose_tolerance, 0.01);
	private_nh.param("idle_timer_duration", _idle_timer_duration, 15.0);
	private_nh.param("unstuck_timer_duration", _unstuck_timer_duration, 5.0);
	private_nh.param("idle_timer_behavior", _idle_timer_behavior, false);

	ros::NodeHandle nh("rsm");
	_get_navigation_goal_service =
			nh.serviceClient<rsm_msgs::GetNavigationGoal>("getNavigationGoal");
	_navigation_goal_completed_service = nh.serviceClient<
			rsm_msgs::GoalCompleted>("navigationGoalCompleted");
	_get_robot_pose_service = nh.serviceClient<rsm_msgs::GetRobotPose>(
			"getRobotPose");
	_get_reverse_mode_service = nh.serviceClient<std_srvs::Trigger>(
			"getReverseMode");
	_reverse_mode_subscriber = nh.subscribe<std_msgs::Bool>("reverseMode", 10,
			&NavigationState::reverseModeCallback, this);
	_get_exploration_mode_service = nh.serviceClient<std_srvs::Trigger>(
			"getExplorationMode");
	_operation_mode_subscriber = nh.subscribe<rsm_msgs::OperationMode>(
			"operationMode", 1, &NavigationState::operationModeCallback, this);
	_idle_timer = _nh.createTimer(ros::Duration(_idle_timer_duration),
			&NavigationState::idleTimerCallback, this, false, false);
	_unstuck_timer = _nh.createTimer(ros::Duration(_unstuck_timer_duration),
			&NavigationState::unstuckTimerCallback, this, false, false);
	//initialize variables
	_name = "Navigation";
	_navigation_mode = -1;
	_goal_active = false;
	_reverse_mode = false;
	_exploration_mode = -1;
	_robot_did_move = false;
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
	_navigation_completed_status = rsm_msgs::GoalStatus::ACTIVE;
	_unstucking_robot = false;
}

void NavigationState::onEntry() {
	//Request navigation goal from Service Provider
	rsm_msgs::GetNavigationGoal srv;
	if (_get_navigation_goal_service.call(srv)) {
		_nav_goal = srv.response.goal;
		_navigation_mode = srv.response.navigationMode;
		_routine = srv.response.routine;
		switch (_navigation_mode) {
		case EXPLORATION:
			_name = "E: Navigation";
			break;
		case WAYPOINT_FOLLOWING:
			_name = "W: Navigation";
			break;
		case SIMPLE_GOAL:
			_name = "G: Navigation";
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
				ros::NodeHandle nh("rsm");
				_get_goal_obsolete_subscriber = nh.subscribe("goalObsolete", 1,
						&NavigationState::goalObsoleteCallback, this);
			}
		} else {
			ROS_ERROR("Failed to call Get Exploration Mode service");
			abortNavigation();
		}
	}
	//Start timer for checking navigation being stuck for too long (15 secs) without proper message from Move Base
	//ROS_WARN_STREAM("Navigation Idle timer started");
	//_idle_timer.start();
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
							_stateinterface->transitionToVolatileState(
									_stateinterface->getPluginState(
									MAPPING_STATE));
							break;
						}
						case WAYPOINT_FOLLOWING: {
							if (_routine.empty()) {
								_navigation_completed_status =
										rsm_msgs::GoalStatus::REACHED;
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
						default: {
							abortNavigation();
							break;
						}
						}
					}
				} else {
					if (!_interrupt_occured) {
						if (!_unstucking_robot) {
							//try reverse navigation
							ROS_INFO_STREAM(
									"Try to unstuck robot by using reversed move base");
							if (_goal_active) {
								_move_base_client->cancelGoal();
							}
							_goal_active = false;
							if (_reverse_mode) {
								_move_base_client.reset(
										new MoveBaseClient("move_base", true));
							} else {
								_move_base_client.reset(
										new MoveBaseClient("move_base_reverse",
												true));
							}
							_unstuck_timer.start();
							_unstucking_robot = true;
						} else {
							//navigation failed
							_navigation_completed_status =
									rsm_msgs::GoalStatus::FAILED;
							switch (_navigation_mode) {
							case EXPLORATION: {
								_stateinterface->transitionToVolatileState(
										_stateinterface->getPluginState(
										CALCULATEGOAL_STATE));
								break;
							}
							case WAYPOINT_FOLLOWING: {
								_stateinterface->transitionToVolatileState(
										boost::make_shared<
												WaypointFollowingState>());
								break;
							}
							default: {
								abortNavigation();
								break;
							}
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
			_idle_timer.start();
		}
	}
}

void NavigationState::onExit() {
	if (_goal_active) {
		_move_base_client->cancelGoal();
	}
	if (_navigation_completed_status != rsm_msgs::GoalStatus::ACTIVE) {
		rsm_msgs::GoalCompleted srv;
		srv.request.status.goal_status = _navigation_completed_status;
		if (!_navigation_goal_completed_service.call(srv)) {
			ROS_ERROR("Failed to call Complete Navigation Goal service");
		}
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
	_navigation_completed_status = rsm_msgs::GoalStatus::ABORTED;
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
		if (_navigation_mode == SIMPLE_GOAL) {
			_stateinterface->transitionToVolatileState(
					boost::make_shared<IdleState>());
		}
		break;
	}
}

void NavigationState::idleTimerCallback(const ros::TimerEvent &event) {
	ROS_ERROR("Navigation aborted because robot appears to be stuck");
	if (_idle_timer_behavior) {	//end navigation
		abortNavigation();
	} else { //declare goal as failed
		_navigation_completed_status = rsm_msgs::GoalStatus::FAILED;
		switch (_navigation_mode) {
		case EXPLORATION: {
			_stateinterface->transitionToVolatileState(
					_stateinterface->getPluginState(
					CALCULATEGOAL_STATE));
			break;
		}
		case WAYPOINT_FOLLOWING: {
			_stateinterface->transitionToVolatileState(
					boost::make_shared<WaypointFollowingState>());
			break;
		}
		default: {
			abortNavigation();
			break;
		}
		}
	}
}

void NavigationState::unstuckTimerCallback(const ros::TimerEvent &event) {
	if (_goal_active) {
		_move_base_client->cancelGoal();
	}
	_goal_active = false;
	if (_reverse_mode) {
		_move_base_client.reset(new MoveBaseClient("move_base_reverse", true));
	} else {
		_move_base_client.reset(new MoveBaseClient("move_base", true));
	}
}

void NavigationState::comparePose() {
	if (_operation_mode == rsm_msgs::OperationMode::AUTONOMOUS) {
		if (_comparison_counter++ >= 10) { //only compare poses every 10th call to reduce load
			tf::Pose current_pose;
			rsm_msgs::GetRobotPose srv;
			if (_get_robot_pose_service.call(srv)) {
				tf::poseMsgToTF(srv.response.pose, current_pose);
				tf::Pose pose_difference = current_pose.inverseTimes(
						_last_pose);
				if (pose_difference.getOrigin().x() < _pose_tolerance
						&& pose_difference.getOrigin().y() < _pose_tolerance
						&& pose_difference.getOrigin().z() < _pose_tolerance
						&& pose_difference.getRotation().x() < _pose_tolerance
						&& pose_difference.getRotation().y() < _pose_tolerance
						&& pose_difference.getRotation().z()
								< _pose_tolerance) {
					_idle_timer.start();
				} else {
					_idle_timer.stop();
					if (!_robot_did_move
							&& _last_pose.getRotation().w() != 0.0) { //not initial
						_robot_did_move = true;
					}
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
		const std_msgs::Bool::ConstPtr &msg) {
	if (msg->data && !_interrupt_occured) {
		_navigation_completed_status = rsm_msgs::GoalStatus::ABORTED;
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(
						_robot_did_move ? MAPPING_STATE : CALCULATEGOAL_STATE));
	}
}

void NavigationState::reverseModeCallback(
		const std_msgs::Bool::ConstPtr &reverse_mode) {
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
		const rsm_msgs::OperationMode::ConstPtr &operation_mode) {
	_operation_mode = operation_mode->mode;
}

void NavigationState::abortNavigation() {
	if (!_interrupt_occured) {
		_navigation_completed_status = rsm_msgs::GoalStatus::ABORTED;
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::NavigationState, rsm::BaseState)
