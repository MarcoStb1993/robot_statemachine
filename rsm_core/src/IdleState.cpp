#include <rsm_core/IdleState.h>

namespace rsm {

IdleState::IdleState() {
}

IdleState::~IdleState() {
}

void IdleState::onSetup() {
	_name = "Idle";
}

void IdleState::onEntry() {
}

void IdleState::onActive() {
}

void IdleState::onExit() {
}

void IdleState::onExplorationStart(bool &success, std::string &message) {
	success = true;
	message = "Exploration started";
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(MAPPING_STATE));
	}
}

void IdleState::onExplorationStop(bool &success, std::string &message) {
	success = false;
	message = "Exploration not running";
}

void IdleState::onWaypointFollowingStart(bool &success, std::string &message) {
	success = true;
	message = "Waypoint following started";
	_stateinterface->transitionToVolatileState(
			boost::make_shared<WaypointFollowingState>());
}

void IdleState::onWaypointFollowingStop(bool &success, std::string &message) {
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
