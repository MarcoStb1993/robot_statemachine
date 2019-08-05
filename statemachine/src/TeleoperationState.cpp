#include <rsm_core/TeleoperationState.h>

namespace rsm {

TeleoperationState::TeleoperationState() {
}

TeleoperationState::~TeleoperationState() {
}

void TeleoperationState::onSetup() {
	_name = "Teleoperation";
}

void TeleoperationState::onEntry() {
}

void TeleoperationState::onActive() {
}

void TeleoperationState::onExit() {
}

void TeleoperationState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Teleoperation active";
}

void TeleoperationState::onExplorationStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration not running";
}

void TeleoperationState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Teleoperation active";
}

void TeleoperationState::onWaypointFollowingStop(bool &success,
		std::string &message) {
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
