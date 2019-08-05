#include "ros/ros.h"
#include "std_srvs/SetBool.h"

bool boot_finished = false;
bool node_finished = false;

bool bootUpService(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (boot_finished) {
		res.success = 1;
		res.message = "Finished";
		node_finished = true;
	} else {
		res.success = false;
		res.message = "Still booting ...";
	}
	return true;
}

void timerCallback(const ros::TimerEvent&) {
	boot_finished = true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "bootUpNode");
	ros::NodeHandle privateNh("~");
	double wait_time;
	privateNh.param<double>("wait_time", wait_time, 1);
	ros::NodeHandle nh("rsm");
	ros::ServiceServer bootup_service = nh.advertiseService("bootUpFinished",
			bootUpService);
	ros::Timer timer = nh.createTimer(ros::Duration(wait_time), timerCallback,
			true);

	while (ros::ok() && !node_finished) {
		ros::spin();
	}
	return 0;
}
