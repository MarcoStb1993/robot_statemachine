#include <rsm_core/EmergencyStopState.h>

namespace rsm {
EmergencyStopState::EmergencyStopState() {
}

EmergencyStopState::~EmergencyStopState() {
}

void EmergencyStopState::onSetup() {
	_name = "Emergency Stop";
}

void EmergencyStopState::onEntry() {
}

void EmergencyStopState::onActive() {
}

void EmergencyStopState::onExit() {
}

void EmergencyStopState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Emergency Stop";
}

void EmergencyStopState::onExplorationStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration not running";
}

void EmergencyStopState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Emergency Stop";
}

void EmergencyStopState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Waypoint following not running";
}

void EmergencyStopState::onInterrupt(int interrupt) {
	if (interrupt == INTERRUPT_END) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
		_interrupt_occured = true;
	}
}

} /* namespace rsm */
