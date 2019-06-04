#include <statemachine_additions/ReversePathState.h>

namespace statemachine {

ReversePathState::ReversePathState() {
	ROS_INFO("ReversePathState constructed");
	_name = "Reverse Path";
}

ReversePathState::~ReversePathState() {
	ROS_INFO("ReversePathState destructed");
}

void ReversePathState::onSetup() {
	ROS_INFO("ReversePathState setup");
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("autonomy_cmd_vel_topic",
			_autonomy_cmd_vel_topic, "/cmd_vel");
	ros::NodeHandle nh("statemachine");
	_get_cmd_vel_recording_service = nh.serviceClient<
			statemachine_msgs::GetCmdVelRecording>("getCmdVelRecording");
	_reset_cmd_vel_recording_service = nh.serviceClient<std_srvs::Trigger>(
			"resetCmdVelRecording");
	_cmd_vel_publisher = _nh.advertise<geometry_msgs::Twist>(
			_autonomy_cmd_vel_topic, 10);

	_cmd_vel_replaying = false;
	double controller_frequency;
	_nh.param("/move_base/controller_frequency", controller_frequency, 20.0);
	_cmd_vel_replay_timer = _nh.createTimer(
			ros::Duration(1 / controller_frequency),
			&ReversePathState::timerCallback, this);
	_cmd_vel_replay_timer.stop();

	statemachine_msgs::GetCmdVelRecording srv;
	if (_get_cmd_vel_recording_service.call(srv)) {
		_cmd_vel_msgs = srv.response.cmdVelMsgs;
		_navigation_mode = srv.response.navigationMode;
		switch (_navigation_mode) {
		case EXPLORATION:
			_name = "Reverse Path: Exploration";
			break;
		case WAYPOINT_FOLLOWING:
			_name = "Reverse Path: Waypoint Following";
			break;
		case SIMPLE_GOAL:
			_name = "Reverse Path: Simple Goal";
			break;
		default:
			_name = "Reverse Path";
			break;
		}
	} else {
		ROS_ERROR("Failed to call Get Cmd Vel Msgs service");
		abortNavigation();
	}
}

void ReversePathState::onEntry() {
	ROS_INFO("ReversePathState entered");
	_cmd_vel_replaying = true;
	_current_cmd_vel_msg = _cmd_vel_msgs.size() - 1;
	_cmd_vel_replay_timer.start();
}

void ReversePathState::onActive() {
	//ROS_INFO("ReversePathState active");
}

void ReversePathState::onExit() {
	ROS_INFO("ReversePathState exited");
	_cmd_vel_replaying = false;
	_cmd_vel_replay_timer.stop();

}

void ReversePathState::onExplorationStart(bool &success, std::string &message) {
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

void ReversePathState::onExplorationStop(bool &success, std::string &message) {
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

void ReversePathState::onWaypointFollowingStart(bool &success,
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

void ReversePathState::onWaypointFollowingStop(bool &success,
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

void ReversePathState::onInterrupt(int interrupt) {
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

void ReversePathState::timerCallback(const ros::TimerEvent& event) {
	if (!_cmd_vel_msgs.empty() && _current_cmd_vel_msg < _cmd_vel_msgs.size()) {
		publishReverseCmdVelMsg();
	} else {
		ROS_INFO("ReversePathState timer finished replaying");
		_cmd_vel_replaying = false;
		_cmd_vel_replay_timer.stop();
		if (!_interrupt_occured) {
			switch (_navigation_mode) {
			case EXPLORATION:
				_stateinterface->transitionToVolatileState(
						_stateinterface->getPluginState(CALCULATEGOAL_STATE));
				break;
			case WAYPOINT_FOLLOWING:
				_stateinterface->transitionToVolatileState(
						boost::make_shared<WaypointFollowingState>());
				break;
			case SIMPLE_GOAL:
				abortNavigation();
				break;
			}
		}
	}
}

void ReversePathState::publishReverseCmdVelMsg() {
	geometry_msgs::Twist reversed_cmd_vel;
	reversed_cmd_vel.linear.x = _cmd_vel_msgs[_current_cmd_vel_msg].linear.x
			* -1;
	reversed_cmd_vel.linear.y = _cmd_vel_msgs[_current_cmd_vel_msg].linear.y
			* -1;
	reversed_cmd_vel.angular.z = _cmd_vel_msgs[_current_cmd_vel_msg].angular.z
			* -1;
	_current_cmd_vel_msg--;
	_cmd_vel_publisher.publish(reversed_cmd_vel);
}

void ReversePathState::abortNavigation() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

} /* namespace statemachine */
