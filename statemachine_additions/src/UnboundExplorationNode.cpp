#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <frontier_exploration/ExploreTaskAction.h>
//#include <exploration_msgs/ExploreAction.h>

float xdim, ydim, yoffset, xoffset, goalx, goaly; //exploration boundary and starting point as parameters

int delay;  //delay in seconds to submitting the service message
bool unbound;   //execute unbound exploration

int main(int argc, char** argv) {
	ros::init(argc, argv, "unboundExploration");

	ros::NodeHandle nh("~");
	goalx = 0.0;
	goaly = 0.0;
	xdim = 0.0;
	ydim = 0.0;
	xoffset = 0.0;
	yoffset = 0.0;
	unbound = false;
	delay = 0;
	nh.getParam("goalx", goalx);
	nh.getParam("goaly", goaly);
	nh.getParam("xdim", xdim);
	nh.getParam("ydim", ydim);
	nh.getParam("xoffset", xoffset);
	nh.getParam("yoffset", yoffset);
	nh.getParam("unbound", unbound);
	nh.getParam("delay", delay);
	//only send service message if delay!=0
	if (delay > 0) {
		ros::Duration(delay).sleep(); //wait delay seconds unit service message is sent
		ROS_INFO("Send polygon!");
		//actionlib::SimpleActionClient<exploration_msgs::ExploreAction> client("explore_server", true);
		actionlib::SimpleActionClient<frontier_exploration::ExploreTaskAction> client("explore_server", true);
		client.waitForServer();
		//exploration_msgs::ExploreGoal goal;
		frontier_exploration::ExploreTaskGoal goal;

		geometry_msgs::PolygonStamped boundary;
		boundary.header.seq = 0;
		boundary.header.stamp = ros::Time::now();
		boundary.header.frame_id = "map";

		//only define boundary points for limited exploration, leave blank for unbound
		if (!unbound) {
			// defining the boundary polygon here
			boundary.polygon.points.reserve(4);
			geometry_msgs::Point32 p1, p2, p3, p4, p5;
			p1.x = xoffset + xdim / 2;
			p1.y = yoffset + ydim / 2;
			p1.z = 0.0;
			p2.x = xoffset + xdim / 2;
			p2.y = yoffset - ydim / 2;
			p2.z = 0.0;
			p3.x = xoffset - xdim / 2;
			p3.y = yoffset - ydim / 2;
			p3.z = 0.0;
			p4.x = xoffset - xdim / 2;
			p4.y = yoffset + ydim / 2;
			p4.z = 0.0;
			boundary.polygon.points.push_back(p1);
			boundary.polygon.points.push_back(p2);
			boundary.polygon.points.push_back(p3);
			boundary.polygon.points.push_back(p4);

		}

		// defining the explore center here
		geometry_msgs::PointStamped start_point;
		start_point.point.x = goalx;
		start_point.point.y = goaly;
		start_point.point.z = 0.0;
		start_point.header.frame_id = "map";
		start_point.header.stamp = ros::Time::now();
		start_point.header.seq = 5;

		//goal.boundary = boundary;
		//goal.start_point = start_point;
		goal.explore_boundary = boundary;
		goal.explore_center = start_point;
		client.sendGoal(goal);
	}
}
