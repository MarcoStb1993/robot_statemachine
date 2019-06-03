#include <statemachine_additions/ReversingRoutineState.h>

namespace statemachine {

ReversingRoutineState::ReversingRoutineState() {
	ROS_INFO("ReversingRoutineState constructed");
	_name = "Mapping";
}

ReversingRoutineState::~ReversingRoutineState() {
	ROS_INFO("ReversingRoutineState destructed");
}

void ReversingRoutineState::onSetup() {
	ROS_INFO("ReversingRoutineState setup");
	ros::NodeHandle nh("statemachine");
	_set_reverse_moving_service = nh.serviceClient < std_srvs::SetBool
			> ("setReverseMode");
	_get_reverse_moving_service = nh.serviceClient < std_srvs::Trigger
			> ("getReverseMode");
	_set_rona_reverse_on = _nh.serviceClient < std_srvs::Empty
			> ("rona/move/set_reverse_on");
	_set_rona_reverse_off = _nh.serviceClient < std_srvs::Empty
			> ("rona/move/set_reverse_off");
	_reverse_mode_active = false;
}

void ReversingRoutineState::onEntry() {
	ROS_INFO("ReversingRoutineState entered");
	std_srvs::Trigger srv;
	if (_get_reverse_moving_service.call(srv)) {
		if (srv.response.success) {
			_reverse_mode_active = true;
		} else {
			_reverse_mode_active = false;
		}
	} else {
		ROS_ERROR("Failed to call Get Reverse Mode service");
		if (!_interrupt_occured) {
			_stateinterface->transitionToVolatileState(
					boost::make_shared<WaypointFollowingState>());
		}
	}
}

void ReversingRoutineState::onActive() {
	//ROS_INFO("ReversingRoutineState active");
	std_srvs::Empty srv;
	if (_reverse_mode_active) {
		if (_set_rona_reverse_off.call(srv)) {
			std_srvs::SetBool srv2;
			srv2.request.data = false;
			if (!_set_reverse_moving_service.call(srv2)) {
				ROS_ERROR("Failed to call Set Reverse Mode service");
			}
		} else {
			ROS_ERROR("Failed to call Set Reverse Mode On service");
		}
	} else {
		if (_set_rona_reverse_on.call(srv)) {
			std_srvs::SetBool srv2;
			srv2.request.data = true;
			if (!_set_reverse_moving_service.call(srv2)) {
				ROS_ERROR("Failed to call Set Reverse Mode service");
			}
		} else {
			ROS_ERROR("Failed to call Set Reverse Mode Off service");
		}
	}
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<WaypointFollowingState>());
	}
}

void ReversingRoutineState::onExit() {
	ROS_INFO("ReversingRoutineState exited");
}

void ReversingRoutineState::onExplorationStart(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Start/Pause called in ReversingRoutineState");
	success = false;
	message = "Waypoint following running";
}

void ReversingRoutineState::onExplorationStop(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Stop called in ReversingRoutineState");
	success = false;
	message = "Waypoint following running";
}

void ReversingRoutineState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/pause called in ReversingRoutineState");
	success = false;
	message = "Waypoint following running";
}

void ReversingRoutineState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in ReversingRoutineState");
	success = true;
	message = "Waypoint following stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void ReversingRoutineState::onInterrupt(int interrupt) {
	if (interrupt == EMERGENCY_STOP_INTERRUPT) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
	} else {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
	}
	_interrupt_occured = true;
}

}

PLUGINLIB_EXPORT_CLASS(statemachine::ReversingRoutineState,
		statemachine::BaseState)
