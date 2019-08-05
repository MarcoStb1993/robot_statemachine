#include "ros/ros.h"
#include <rsm_core/StateInterface.h>
#include <rsm_core/BootState.h>

boost::shared_ptr<rsm::StateInterface> stateInterface;

void loopCallback(const ros::TimerEvent&) {
	stateInterface->awake();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "rsmNode");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 20.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	stateInterface.reset(new rsm::StateInterface());
	stateInterface->awake();
	stateInterface->transitionToVolatileState(
			boost::make_shared<rsm::BootState>());
	stateInterface->awake();
	loop_timer.start();
	ros::spin();
	return 0;
}

