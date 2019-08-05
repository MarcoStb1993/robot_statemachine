#include <rsm_additions/ReversingRoutineState.h>

namespace rsm {

ReversingRoutineState::ReversingRoutineState() {
}

ReversingRoutineState::~ReversingRoutineState() {
}

void ReversingRoutineState::onSetup() {

	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rsm");
	_set_reverse_moving_service = nh.serviceClient<std_srvs::SetBool>(
			"setReverseMode");
	_get_reverse_moving_service = nh.serviceClient<std_srvs::Trigger>(
			"getReverseMode");
	//initialize variables
	_name = "Mapping";
	_reverse_mode_active = false;
}

void ReversingRoutineState::onEntry() {
	//Request current reverse mode status from Service Provider
	std_srvs::Trigger srv;
	if (_get_reverse_moving_service.call(srv)) {
		_reverse_mode_active = srv.response.success;
	} else {
		ROS_ERROR("Failed to call Get Reverse Mode service");
		if (!_interrupt_occured) {
			_stateinterface->transitionToVolatileState(
					boost::make_shared<WaypointFollowingState>());
		}
	}
}

void ReversingRoutineState::onActive() {
	//Toggle reverse mode
	std_srvs::SetBool srv;
	srv.request.data = !_reverse_mode_active;
	if (!_set_reverse_moving_service.call(srv)) {
		ROS_ERROR("Failed to call Set Reverse Mode service");
	}
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<WaypointFollowingState>());
	}
}

void ReversingRoutineState::onExit() {
}

void ReversingRoutineState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following running";
}

void ReversingRoutineState::onExplorationStop(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following running";
}

void ReversingRoutineState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following running";
}

void ReversingRoutineState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = true;
	message = "Waypoint following stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void ReversingRoutineState::onInterrupt(int interrupt) {
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

}

PLUGINLIB_EXPORT_CLASS(rsm::ReversingRoutineState,
		rsm::BaseState)
