#include <rsm_core/BootState.h>

namespace rsm {

BootState::BootState() {
}

BootState::~BootState() {
}

void BootState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rsm");
	_bootupClient = nh.serviceClient<std_srvs::SetBool>("bootUpFinished");
	//initialize variables
	_name = "Boot";
}

void BootState::onEntry() {
}

void BootState::onActive() {
	//Request if boot was finished
	std_srvs::SetBool srv;
	srv.request.data = true;
	if (_bootupClient.call(srv)) {
		if (srv.response.success) {
			_stateinterface->transitionToVolatileState(
					boost::make_shared<IdleState>());
		}
	} else {
		ROS_ERROR("Failed to call service Bootup Finished");
	}
}

void BootState::onExit() {
}

void BootState::onExplorationStart(bool &success, std::string &message) {
	success = false;
	message = "Still booting";
}

void BootState::onExplorationStop(bool &success, std::string &message) {
	success = false;
	message = "Still booting";
}

void BootState::onWaypointFollowingStartStop(bool &success,
		std::string &message) {
	success = false;
	message = "Still booting";
}

void BootState::onInterrupt(int interrupt) {
	if (interrupt == EMERGENCY_STOP_INTERRUPT) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
		_interrupt_occured = true;
	}
}

}
