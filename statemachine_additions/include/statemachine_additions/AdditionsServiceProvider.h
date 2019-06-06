#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <statemachine_msgs/GetCmdVelRecording.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <boost/circular_buffer.hpp>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

namespace statemachine {

class AdditionsServiceProvider {

public:
	AdditionsServiceProvider();
	~AdditionsServiceProvider();
	void publishTopics();

private:
	ros::NodeHandle _nh;

	ros::ServiceServer _start_stop_cmd_vel_recording_service;
	ros::ServiceServer _get_cmd_vel_recording_service;
	ros::ServiceServer _reset_cmd_vel_recording_service;
	ros::ServiceServer _request_reverse_path_usage_service;
	ros::ServiceServer _reset_reverse_path_usage_service;
	ros::Subscriber _reverse_path_cmd_vel_subscriber;

	ros::ServiceClient _set_rona_reverse_on;
	ros::ServiceClient _set_rona_reverse_off;

	ros::ServiceServer _set_reverse_mode_service;
	ros::ServiceServer _get_reverse_mode_service;
	ros::Publisher _reverse_mode_publisher;
	ros::Subscriber _reverse_mode_cmd_vel_subscriber;
	ros::Publisher _reverse_mode_cmd_vel_publisher;

	bool _reverse_path_used;
	boost::circular_buffer<geometry_msgs::Twist> _cmd_vel_msgs;
	int _msg_buffer_size;
	std::string _autonomy_cmd_vel_topic;

	MoveBaseActionServer* as;

	ros::Subscriber frontiers_marker_array_subscriber;
	ros::Publisher frontier_poses_publisher;

	/**
	 * Is currently driving in reverse
	 */
	bool _reverse_mode_active;
	/**
	 * Is the Navigation stack used as Plugin for navigation
	 */
	bool _navigation_plugin_used;

	/**
	 * List of all extracted frontier centers
	 */
	geometry_msgs::PoseArray _frontier_poses;

	bool startStopCmdVelRecording(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	bool getCmdVelRecording(statemachine_msgs::GetCmdVelRecording::Request &req,
			statemachine_msgs::GetCmdVelRecording::Response &res);
	bool resetCmdVelRecording(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool requestReversePathUsage(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool resetReversePathUsage(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);

	bool setReverseMode(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	bool getReverseMode(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	void publishReverseMode();
	void reverseModeCmdVelCallback(
			const geometry_msgs::Twist::ConstPtr& cmd_vel);
	bool setReverseModeRona(bool on);

	void navigationGoalCallback(
			const move_base_msgs::MoveBaseGoalConstPtr& frontier_goal);

	void frontierCallback(
			const visualization_msgs::MarkerArray::ConstPtr& frontiers);
	void publishFrontierPoses();
};

}
