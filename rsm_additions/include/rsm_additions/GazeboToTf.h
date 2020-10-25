/*
 * GazeboToTf.h
 *
 *  Created on: Oct 25, 2020
 *      Author: marco
 */

#ifndef GAZEBOTOTF_H_
#define GAZEBOTOTF_H_

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace rsm {

class GazeboToTf {
public:
	GazeboToTf();
	virtual ~GazeboToTf();
private:
	std::string _odom_topic, _map_frame, _robot_frame;
	ros::NodeHandle _nh;
	ros::Subscriber _odom_subscriber;
	tf2_ros::TransformBroadcaster _tf_broadcaster;

	void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
};

}

#endif /* GAZEBOTOTF_H_ */
