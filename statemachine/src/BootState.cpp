#include "statemachine/BootState.h"

namespace statemachine {

BootState::BootState() {
	ROS_INFO("BootState constructed");
	_name = "Boot";
	ros::NodeHandle nh("statemachine");
	_bootupClient = nh.serviceClient<std_srvs::SetBool>("bootUpFinished");
}

BootState::~BootState() {
	ROS_INFO("BootState destructed");
}

void BootState::onSetup() {
	ROS_INFO("BootState setup");
}

void BootState::onEntry() {
	ROS_INFO("BootState entered");
}

void BootState::onActive() {
	//ROS_INFO("BootState active");
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
	ROS_INFO("BootState exited");
}

void BootState::onExplorationStart(bool &success, std::string &message) {
	ROS_INFO("Exploration Start called in BootState");
	success = false;
	message = "Still booting";
}

void BootState::onExplorationStop(bool &success, std::string &message) {
	ROS_INFO("Exploration Stop called in BootState");
	success = false;
	message = "Still booting";
}

void BootState::onWaypointFollowingStartStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/stop called in BootState");
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
