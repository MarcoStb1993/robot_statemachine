#include "ros/ros.h"

#include <rsm_additions/AdditionsServiceProvider.h>

boost::shared_ptr<rsm::AdditionsServiceProvider> service_provider;

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
			new rsm::AdditionsServiceProvider());
	ros::spin();
	service_provider.reset();
	return 0;
}
