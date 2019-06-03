#include <statemachine_additions/RonaNavigationState.h>

namespace statemachine {

RonaNavigationState::RonaNavigationState() {
	ROS_INFO("RonaNavigationState constructed");
	_name = "Navigation";
}

RonaNavigationState::~RonaNavigationState() {
	ROS_INFO("RonaNavigationState destructed");
}

void RonaNavigationState::onSetup() {
	ROS_INFO("RonaNavigationState setup");
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

	_waypoint_following = false;
	_goal_active = false;
	_comparison_counter = 0;
	_sirona_state.state = -1;
	_nav_goal_published_counter = 0;

	_idle_timer = nh.createTimer(ros::Duration(30.0),
			&RonaNavigationState::timerCallback, this, true); //15 seconds idle timer
	_idle_timer.stop();

	_sirona_state_subscriber = _nh.subscribe("rona/sirona/state", 1,
			&RonaNavigationState::sironaStateCallback, this);
	_nav_goal_publisher = _nh.advertise<geometry_msgs::PoseStamped>(
			"rona/exploration/target", 1, true);
	_nav_stop_publisher = _nh.advertise<std_msgs::Bool>("rona/move/pause", 1,
			true);

	_get_exploration_mode = nh.serviceClient<std_srvs::Trigger>(
			"getExplorationMode");
	_exploration_mode = 1; //finish goals

}

void RonaNavigationState::onEntry() {
	ROS_INFO("RonaNavigationState entered");
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
	if (!_waypoint_following) {
		std_srvs::Trigger srv2;
		if (_get_exploration_mode.call(srv2)) {
			_exploration_mode = srv2.response.success;
			if (_exploration_mode) {
				_get_goal_obsolete = _nh.subscribe("statemachine/goalObsolete",
						1, &RonaNavigationState::goalObsoleteCallback, this);
			}
		} else {
			ROS_ERROR("Failed to call Get Exploration Mode service");
			abortNavigation();
		}
	}
}

void RonaNavigationState::onActive() {
//ROS_INFO("RonaNavigationState active");
	if (_goal_active) {
		double passed_time = (ros::Time::now() - _nav_start_time).toSec();
		if (passed_time > WAIT_TIME) {
			if (_sirona_state.state == _sirona_state.ARRIVED) {
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
									boost::make_shared<WaypointFollowingState>());
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
			} else if (_sirona_state.state == _sirona_state.ABORTED
					|| _sirona_state.state == _sirona_state.UNREACHABLE) {
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
						//ROS_INFO("Leave RonaNavigationState with %lu failed goals",_failed_goals.size());
						statemachine_msgs::AddFailedGoal srv;
						srv.request.failedGoal = _nav_goal;
						if (!_add_failed_goal_service.call(srv)) {
							ROS_ERROR("Failed to call Add Failed Goal service");
						}
						_stateinterface->transitionToVolatileState(
								_stateinterface->getPluginState(
								CALCULATEGOAL_STATE));
					}
				}
			} else {
				comparePose();
			}
		}
	} else {
		geometry_msgs::PoseStamped target;
		target.header.frame_id = "map";
		target.header.stamp = ros::Time::now();
		target.pose = _nav_goal;
		_nav_goal_publisher.publish(target);
		_nav_start_time = ros::Time::now();
		_goal_active = true;
	}
}

void RonaNavigationState::onExit() {
	ROS_INFO("RonaNavigationState exited");
	std_msgs::Bool msg;
	msg.data = true;
	_nav_stop_publisher.publish(msg);
}

void RonaNavigationState::onExplorationStart(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Start called in RonaRonaNavigationState");
	if (_waypoint_following) {
		success = false;
		message = "Waypoint following running";
	} else {
		success = false;
		message = "Exploration running";
	}
}

void RonaNavigationState::onExplorationStop(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Stop called in RonaRonaNavigationState");
	if (_waypoint_following) {
		success = false;
		message = "Waypoint following running";
	} else {
		success = true;
		message = "Exploration stopped";
		abortNavigation();
	}
}

void RonaNavigationState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO(
			"Waypoint following start/pause called in RonaRonaNavigationState");
	success = false;
	if (_waypoint_following) {
		message = "Waypoint following running";
	} else {
		message = "Exploration running";
	}
}

void RonaNavigationState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in RonaRonaNavigationState");
	if (_waypoint_following) {
		success = true;
		message = "Waypoint following stopped";
		abortNavigation();
	} else {
		success = false;
		message = "Exploration running";
	}
}

void RonaNavigationState::onInterrupt(int interrupt) {
	if (interrupt == EMERGENCY_STOP_INTERRUPT) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
	} else {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
	}
	_interrupt_occured = true;
}

void RonaNavigationState::timerCallback(const ros::TimerEvent& event) {
	ROS_INFO("Navigation Timeout!");
	abortNavigation();
}

void RonaNavigationState::comparePose() {
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

void RonaNavigationState::sironaStateCallback(const rona_msgs::State& msg) {
	_sirona_state = msg;
}

void RonaNavigationState::goalObsoleteCallback(
		const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data && !_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(
				CALCULATEGOAL_STATE));
	}
}

void RonaNavigationState::abortNavigation() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(statemachine::RonaNavigationState,
		statemachine::BaseState)
