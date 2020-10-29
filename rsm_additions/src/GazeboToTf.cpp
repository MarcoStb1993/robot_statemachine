/*
 * GazeboToTf.cpp
 *
 *  Created on: Oct 25, 2020
 *      Author: marco
 */

#include <rsm_additions/GazeboToTf.h>

namespace rsm {

GazeboToTf::GazeboToTf() {
	ros::NodeHandle privateNh("~");
	privateNh.param < std::string
			> ("odom_topic", _odom_topic, "/ground_truth/state");
	privateNh.param < std::string > ("map_frame", _map_frame, "map");
	privateNh.param < std::string
			> ("robot_frame", _robot_frame, "robot_footprint");

	_odom_subscriber = _nh.subscribe(_odom_topic, 10, &GazeboToTf::odomCallback,
			this);
}

GazeboToTf::~GazeboToTf() {
	// TODO Auto-generated destructor stub
}

void GazeboToTf::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now();
	transformStamped.header.frame_id = _map_frame;
	transformStamped.child_frame_id = _robot_frame;
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
	transformStamped.transform.translation.y = msg->pose.pose.position.y;
	transformStamped.transform.translation.z = 0;
	transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

	_tf_broadcaster.sendTransform(transformStamped);
}

}

