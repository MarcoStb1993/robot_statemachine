#include "ros/ros.h"

#include <statemachine_additions/AdditionsServiceProvider.h>

boost::shared_ptr<statemachine::AdditionsServiceProvider> service_provider;

void loopCallback(const ros::TimerEvent&) {
	service_provider->publishTopics();
}
int main(int argc, char **argv) {
	ros::init(argc, argv, "additionsServiceProvider");
	ros::NodeHandle private_nh("~");
	double loop_rate;
	private_nh.param("update_frequency", loop_rate, 20.0);
	ros::Timer loop_timer = private_nh.createTimer(ros::Duration(1 / loop_rate),
			loopCallback);
	service_provider.reset(
			new statemachine::AdditionsServiceProvider());
	ros::spin();
	return 0;
}
