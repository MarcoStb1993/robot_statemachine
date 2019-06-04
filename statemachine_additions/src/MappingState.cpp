#include <statemachine_additions/MappingState.h>

namespace statemachine {

MappingState::MappingState() {
	ROS_INFO("MappingState constructed");
	_name = "Mapping";
}

MappingState::~MappingState() {
	ROS_INFO("MappingState destructed");
}

void MappingState::onSetup() {
	ROS_INFO("MappingState setup");
}

void MappingState::onEntry() {
	ROS_INFO("MappingState entered");
}

void MappingState::onActive() {
	//ROS_INFO("MappingState active");
	//do mapping stuff
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(CALCULATEGOAL_STATE));
	}
}

void MappingState::onExit() {
	ROS_INFO("MappingState exited");
}

void MappingState::onExplorationStart(bool &success, std::string &message) {
	ROS_INFO("Exploration Start/Pause called in MappingState");
	success = false;
	message = "Exploration running";
}

void MappingState::onExplorationStop(bool &success, std::string &message) {
	ROS_INFO("Exploration Stop called in MappingState");
	success = true;
	message = "Exploration stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void MappingState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/pause called in MappingState");
	success = false;
	message = "Exploration running";
}

void MappingState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in MappingState");
	success = false;
	message = "Exploration running";
}

void MappingState::onInterrupt(int interrupt) {
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

PLUGINLIB_EXPORT_CLASS(statemachine::MappingState, statemachine::BaseState)
