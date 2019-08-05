#include "ros/ros.h"
#include <rsm_rviz_plugins/WaypointFollowingVisualization.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "waypointFollowingVisualizationNode");
	ros::NodeHandle private_nh("~");
	double loop_rate_param;
	private_nh.param("update_frequency", loop_rate_param, 20.0);
	ros::Rate* loop_rate = new ros::Rate(loop_rate_param);
	rsm::WaypointFollowingVisualization waypoint_following_visualization;
	while(ros::ok())
	    {
	        ros::spinOnce();
	        loop_rate->sleep();
	    }
	return 0;
}
