#include <rsm_additions/MappingDummyState.h>

namespace rsm {

MappingDummyState::MappingDummyState() {
}

MappingDummyState::~MappingDummyState() {
}

void MappingDummyState::onSetup() {
	_name = "Mapping";
}

void MappingDummyState::onEntry() {
}

void MappingDummyState::onActive() {
	//do mapping stuff
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(CALCULATEGOAL_STATE));
	}
}

void MappingDummyState::onExit() {
}

void MappingDummyState::onExplorationStart(bool &success, std::string &message) {
	success = false;
	message = "Exploration running";
}

void MappingDummyState::onExplorationStop(bool &success, std::string &message) {
	success = true;
	message = "Exploration stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void MappingDummyState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void MappingDummyState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void MappingDummyState::onInterrupt(int interrupt) {
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

PLUGINLIB_EXPORT_CLASS(rsm::MappingDummyState, rsm::BaseState)
