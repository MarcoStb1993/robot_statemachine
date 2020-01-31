#include "ros/ros.h"
#include <rsm_core/RobotControlMux.h>

boost::shared_ptr<rsm::RobotControlMux> robot_control_mux;

void loopCallback(const ros::TimerEvent&) {
	robot_control_mux->publishTopics();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "robotControlMuxNode");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 20.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	robot_control_mux.reset(new rsm::RobotControlMux());
	ros::spin();
	robot_control_mux.reset();
	return 0;
}
