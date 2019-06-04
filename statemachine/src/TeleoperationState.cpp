#include "statemachine/TeleoperationState.h"

namespace statemachine {

TeleoperationState::TeleoperationState() {
	ROS_INFO("TeleoperationState constructed");
	_name = "Teleoperation";
}

TeleoperationState::~TeleoperationState() {
	ROS_INFO("TeleoperationState destructed");
}

void TeleoperationState::onSetup() {
	ROS_INFO("TeleoperationState setup");
}

void TeleoperationState::onEntry() {
	ROS_INFO("TeleoperationState entered");
}

void TeleoperationState::onActive() {
	//ROS_INFO("TeleoperationState active");
}

void TeleoperationState::onExit() {
	ROS_INFO("TeleoperationState exited");
}

void TeleoperationState::onExplorationStart(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Start called in TeleoperationState");
	success = false;
	message = "Teleoperation active";
}

void TeleoperationState::onExplorationStop(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Stop called in TeleoperationState");
	success = false;
	message = "Exploration not running";
}

void TeleoperationState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/pause called in TeleoperationState");
	success = false;
	message = "Teleoperation active";
}

void TeleoperationState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in TeleoperationState");
	success = false;
	message = "Waypoint following not running";
}

void TeleoperationState::onInterrupt(int interrupt) {
	if (interrupt == EMERGENCY_STOP_INTERRUPT) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
		_interrupt_occured = true;
	} else if (interrupt == INTERRUPT_END) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
		_interrupt_occured = true;
	}

}

}
