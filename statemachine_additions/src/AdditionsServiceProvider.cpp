#include <statemachine_additions/AdditionsServiceProvider.h>

namespace statemachine {

AdditionsServiceProvider::AdditionsServiceProvider() :
		as(NULL) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("autonomy_cmd_vel_topic",
			_autonomy_cmd_vel_topic, "/autonomy/cmd_vel");
	std::string navigation_plugin;
	private_nh.param<std::string>("navigation_plugin", navigation_plugin, "");
	if (navigation_plugin.compare("statemachine::NavigationState") == 0) {
		_navigation_plugin_used = true;
	} else {
		_navigation_plugin_used = false;
	}

	ros::NodeHandle nh("statemachine");
	_start_stop_cmd_vel_recording_service = nh.advertiseService(
			"startStopCmdVelRecording",
			&AdditionsServiceProvider::startStopCmdVelRecording, this);
	_get_cmd_vel_recording_service = nh.advertiseService("getCmdVelRecording",
			&AdditionsServiceProvider::getCmdVelRecording, this);
	_request_reverse_path_usage_service = nh.advertiseService(
			"requestReversePathUsage",
			&AdditionsServiceProvider::requestReversePathUsage, this);
	_reset_reverse_path_usage_service = nh.advertiseService(
			"resetReversePathUsage",
			&AdditionsServiceProvider::resetReversePathUsage, this);
	_reset_cmd_vel_recording_service = nh.advertiseService(
			"resetCmdVelRecording",
			&AdditionsServiceProvider::resetCmdVelRecording, this);

	if (_navigation_plugin_used) {
		_set_navigation_to_reverse_service = nh.advertiseService(
				"setNavigationToReverse",
				&AdditionsServiceProvider::setNavigationToReverse, this);
		std::ostringstream autonomy_cmd_vel_reverse_topic;
		autonomy_cmd_vel_reverse_topic << _autonomy_cmd_vel_topic << "_reverse";
		_reverse_mode_cmd_vel_subscriber = _nh.subscribe(
				autonomy_cmd_vel_reverse_topic.str(), 10,
				&AdditionsServiceProvider::reverseModeCmdVelCallback, this);
		_reverse_mode_cmd_vel_publisher = _nh.advertise<geometry_msgs::Twist>(
				_autonomy_cmd_vel_topic, 10);
	}

	as = new MoveBaseActionServer(_nh, "frontier_move_base",
			boost::bind(&AdditionsServiceProvider::navigationGoalCallback, this,
					_1), false);
	as->start();

	frontiers_marker_array_subscriber = _nh.subscribe("explore/frontiers", 1,
			&AdditionsServiceProvider::frontierCallback, this);
	frontier_poses_publisher = nh.advertise<geometry_msgs::PoseArray>(
			"frontiers", 1);

	double controller_frequency;
	_nh.param("/move_base/controller_frequency", controller_frequency, 20.0);
	_msg_buffer_size = controller_frequency * 2; //2 seconds recorded cmd vel messages
	_cmd_vel_msgs.set_capacity(_msg_buffer_size);
	_reverse_path_used = false;
}

AdditionsServiceProvider::~AdditionsServiceProvider() {

}

void AdditionsServiceProvider::publishTopics() {
	publishFrontierPoses();
}

bool AdditionsServiceProvider::startStopCmdVelRecording(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	if (req.data) {
		_reverse_path_cmd_vel_subscriber = _nh.subscribe(
				_autonomy_cmd_vel_topic, 10,
				&AdditionsServiceProvider::cmdVelCallback, this);
		res.message = "Started Cmd Vel recording";
	} else {
		_reverse_path_cmd_vel_subscriber.shutdown();
		res.message = "Stopped Cmd Vel recording";
	}
	res.success = 1;
	return true;
}

bool AdditionsServiceProvider::getCmdVelRecording(
		statemachine_msgs::GetCmdVelRecording::Request &req,
		statemachine_msgs::GetCmdVelRecording::Response &res) {
	std::vector<geometry_msgs::Twist> cmd_vel_msgs;
	for (int i = 0; i < _cmd_vel_msgs.size(); i++) {
		cmd_vel_msgs.push_back(_cmd_vel_msgs[i]);
	}
	res.cmdVelMsgs = cmd_vel_msgs;
	return true;
}

bool AdditionsServiceProvider::resetCmdVelRecording(
		std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	_cmd_vel_msgs.clear();
	res.success = 1;
	res.message = "Cmd Vel recording reset";
	return true;
}

bool AdditionsServiceProvider::requestReversePathUsage(
		std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	if (!_reverse_path_used) {
		res.success = 1;
		res.message = "Reverse Path available, used now";
		_reverse_path_used = true;
		ROS_INFO("Reverse Path Used");
	} else {
		res.success = 0;
		res.message = "Reverse Path not available";
	}
	return true;
}

bool AdditionsServiceProvider::resetReversePathUsage(
		std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	if (_reverse_path_used) {
		res.success = 1;
		res.message = "Reverse Path Usage reset";
		_reverse_path_used = false;
		ROS_INFO("Reverse Path Available");
	} else {
		res.success = 0;
		res.message = "Reverse Path was available";
	}
	return true;
}

void AdditionsServiceProvider::cmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& msg) {
	if (msg->linear.x != 0 || msg->linear.y != 0 || msg->linear.z != 0
			|| msg->angular.x != 0 || msg->angular.y != 0
			|| msg->angular.z != 0) {
		_cmd_vel_msgs.push_back(*msg);
	}
}

bool AdditionsServiceProvider::setNavigationToReverse(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	res.success = 1;
	res.message = "Mode set";
	return true;
}

void AdditionsServiceProvider::reverseModeCmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	ROS_INFO("message reversed");
	geometry_msgs::Twist cmd_vel_reversed;
	cmd_vel_reversed.linear.x = cmd_vel->linear.x * -1;
	cmd_vel_reversed.linear.y = cmd_vel->linear.y * -1;
	cmd_vel_reversed.linear.z = cmd_vel->linear.z * -1;
	cmd_vel_reversed.angular.x = cmd_vel->angular.x;
	cmd_vel_reversed.angular.y = cmd_vel->angular.y;
	cmd_vel_reversed.angular.z = cmd_vel->angular.z;
	_reverse_mode_cmd_vel_publisher.publish(cmd_vel_reversed);
}

void AdditionsServiceProvider::navigationGoalCallback(
		const move_base_msgs::MoveBaseGoalConstPtr& frontier_goal) {
	as->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
}

void AdditionsServiceProvider::publishFrontierPoses() {
	_frontier_poses.header.frame_id = "map";
	_frontier_poses.header.stamp = ros::Time::now();
	frontier_poses_publisher.publish(_frontier_poses);
}

void AdditionsServiceProvider::frontierCallback(
		const visualization_msgs::MarkerArray::ConstPtr& frontiers) {
	_frontier_poses.poses.clear();
	for (auto it : frontiers->markers) {
		if (it.type == visualization_msgs::Marker::POINTS) {
			int frontier_size = it.points.size();
			if (frontier_size > 0) {
				geometry_msgs::Pose point;
				point.position.x = it.points[frontier_size / 2].x;
				point.position.y = it.points[frontier_size / 2].y;
				point.position.z = it.points[frontier_size / 2].z;
				point.orientation.w = 1.0;
				_frontier_poses.poses.push_back(point);
			}
		}
	}
}
}
