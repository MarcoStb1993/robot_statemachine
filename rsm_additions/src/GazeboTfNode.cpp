#include "ros/ros.h"
#include <rsm_additions/GazeboToTf.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "gazeboTfNode");
	boost::shared_ptr<rsm::GazeboToTf> gazeboToTf;
	gazeboToTf.reset(new rsm::GazeboToTf());
	ros::spin();
	gazeboToTf.reset();
	return 0;
}
