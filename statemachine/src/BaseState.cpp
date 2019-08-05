#include <rsm_core/BaseState.h>

namespace rsm {

BaseState::BaseState() {
	_stateinterface = NULL;
	_interrupt_occured = 0;
}

BaseState::~BaseState() {

}

void BaseState::setStateInterface(StateInterface* stateinterface) {
	_stateinterface = stateinterface;
}

StateInterface* BaseState::getStateInterface() {
	return _stateinterface;
}

std::string BaseState::getName() {
	return _name;
}

}
