#include "ros/ros.h"

#include <statemachine_rona_additions/AdditionsServiceProvider.h>

boost::shared_ptr<statemachine::AdditionsServiceProvider> service_provider;

int main(int argc, char **argv) {
	ros::init(argc, argv, "ronaAdditionsServiceProvider");
	ros::NodeHandle nh;
	service_provider.reset(new statemachine::AdditionsServiceProvider());
	ros::spin();
	return 0;
}
