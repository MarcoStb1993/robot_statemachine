#include "statemachine/IdleState.h"

namespace statemachine {

IdleState::IdleState() {
	ROS_INFO("IdleState constructed");
	_name = "Idle";
}

IdleState::~IdleState() {
	ROS_INFO("IdleState destructed");
}

void IdleState::onSetup() {
	ROS_INFO("IdleState setup");
}

void IdleState::onEntry() {
	ROS_INFO("IdleState entered");
}

void IdleState::onActive() {
	//ROS_INFO("IdleState active");
}

void IdleState::onExit() {
	ROS_INFO("IdleState exited");
}

void IdleState::onExplorationStart(bool &success, std::string &message) {
	ROS_INFO("Exploration Start called in IdleState");
	success = true;
	message = "Exploration started";
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(CALCULATEGOAL_STATE));
	}
}

void IdleState::onExplorationStop(bool &success, std::string &message) {
	ROS_INFO("Exploration Stop called in IdleState");
	success = false;
	message = "Exploration not running";
}

void IdleState::onWaypointFollowingStart(bool &success, std::string &message) {
	ROS_INFO("Waypoint following start/pause called in IdleState");
	success = true;
	message = "Waypoint following started";
	_stateinterface->transitionToVolatileState(
			boost::make_shared<WaypointFollowingState>());
}

void IdleState::onWaypointFollowingStop(bool &success, std::string &message) {
	ROS_INFO("Waypoint following stop called in IdleState");
	success = false;
	message = "Waypoint following not running";
}

void IdleState::onInterrupt(int interrupt) {
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
