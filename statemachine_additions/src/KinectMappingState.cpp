#include <statemachine_additions/KinectMappingState.h>

namespace statemachine {

KinectMappingState::KinectMappingState() {
}

KinectMappingState::~KinectMappingState() {
}

void KinectMappingState::onSetup() {
	_joint_states_subscriber = _nh.subscribe(
			"joint_states", 10, &KinectMappingState::jointStateCallback, this);
	_kinetic_joint_controller = _nh.advertise<std_msgs::Float64>("kinetic_controller/command", 1);
	_name = "Mapping";
	_swivel_state = -1;
}

void KinectMappingState::onEntry() {
	_swivel_state = MOVE_LEFT;
}

void KinectMappingState::onActive() {
	//do mapping stuff
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(CALCULATEGOAL_STATE));
	}
}

void KinectMappingState::onExit() {
}

void KinectMappingState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void KinectMappingState::onExplorationStop(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void KinectMappingState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void KinectMappingState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void KinectMappingState::onInterrupt(int interrupt) {
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

PLUGINLIB_EXPORT_CLASS(statemachine::KinectMappingState,
		statemachine::BaseState)
