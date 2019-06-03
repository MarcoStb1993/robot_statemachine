#include <statemachine/EmergencyStopState.h>

namespace statemachine {
EmergencyStopState::EmergencyStopState() {
	ROS_INFO("EmergencyStopState constructed");
	_name = "Emergency Stop";
}

EmergencyStopState::~EmergencyStopState() {
	ROS_INFO("EmergencyStopState destructed");
}

void EmergencyStopState::onSetup() {
	ROS_INFO("EmergencyStopState setup");
}

void EmergencyStopState::onEntry() {
	ROS_INFO("EmergencyStopState entered");
}

void EmergencyStopState::onActive() {
	//ROS_INFO("EmergencyStopState active");
}

void EmergencyStopState::onExit() {
	ROS_INFO("EmergencyStopState exited");
}

void EmergencyStopState::onExplorationStart(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Start called in EmergencyStopState");
	success = false;
	message = "Emergency Stop";
}

void EmergencyStopState::onExplorationStop(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Stop called in EmergencyStopState");
	success = false;
	message = "Exploration not running";
}

void EmergencyStopState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/pause called in EmergencyStopState");
	success = false;
	message = "Emergency Stop";
}

void EmergencyStopState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in EmergencyStopState");
	success = false;
	message = "Waypoint following not running";
}

void EmergencyStopState::onInterrupt(int interrupt) {
	if (interrupt == INTERRUPT_END) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
	_interrupt_occured = true;
}

} /* namespace statemachine */
