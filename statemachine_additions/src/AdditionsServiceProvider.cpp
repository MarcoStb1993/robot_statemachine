#include <rsm_additions/AdditionsServiceProvider.h>

namespace rsm {

AdditionsServiceProvider::AdditionsServiceProvider() :
		as(NULL) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("autonomy_cmd_vel_topic",
			_autonomy_cmd_vel_topic, "/autonomy/cmd_vel");
	std::string navigation_plugin;
	private_nh.param<std::string>("navigation_plugin", navigation_plugin, "");
	if (navigation_plugin.compare("rsm::NavigationState") == 0) {
		_navigation_plugin_used = true;
	} else {
		_navigation_plugin_used = false;
	}

	ros::NodeHandle nh("rsm");

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

	std::string mapping_plugin;
	private_nh.param<std::string>("mapping_plugin", mapping_plugin, "");
	if (mapping_plugin.compare("rsm::KinectMappingState") == 0) {
		_reset_kinect_position_serivce = nh.advertiseService(
				"resetKinectPosition",
				&AdditionsServiceProvider::resetKinectPosition, this);
		_kinetic_joint_controller = _nh.advertise<std_msgs::Float64>(
				"kinetic_controller/command", 1, true);
	}

	as = new MoveBaseActionServer(_nh, "frontier_move_base",
			boost::bind(&AdditionsServiceProvider::navigationGoalCallback, this,
					_1), false);
	as->start();

	frontiers_marker_array_subscriber = _nh.subscribe("explore/frontiers", 1,
			&AdditionsServiceProvider::frontierCallback, this);
	exploration_goals_publisher = nh.advertise<geometry_msgs::PoseArray>(
			"explorationGoals", 1);
}

AdditionsServiceProvider::~AdditionsServiceProvider() {

}

void AdditionsServiceProvider::publishTopics() {
	publishExplorationGoals();
}

bool AdditionsServiceProvider::setNavigationToReverse(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	res.success = 1;
	res.message = "Mode set";
	return true;
}

void AdditionsServiceProvider::reverseModeCmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& cmd_vel) {
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

void AdditionsServiceProvider::publishExplorationGoals() {
	_exploration_goals.header.frame_id = "map";
	_exploration_goals.header.stamp = ros::Time::now();
	exploration_goals_publisher.publish(_exploration_goals);
}

void AdditionsServiceProvider::frontierCallback(
		const visualization_msgs::MarkerArray::ConstPtr& frontiers) {
	_exploration_goals.poses.clear();
	for (auto it : frontiers->markers) {
		if (it.type == visualization_msgs::Marker::POINTS) {
			int frontier_size = it.points.size();
			if (frontier_size > 0) {
				geometry_msgs::Pose point;
				point.position.x = it.points[frontier_size / 2].x;
				point.position.y = it.points[frontier_size / 2].y;
				point.position.z = it.points[frontier_size / 2].z;
				point.orientation.w = 1.0;
				_exploration_goals.poses.push_back(point);
			}
		}
	}
}

bool AdditionsServiceProvider::resetKinectPosition(
		std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	std_msgs::Float64 kinetic_command;
	kinetic_command.data = 0.0;	//center position of kinect camera
	_kinetic_joint_controller.publish(kinetic_command);
	return true;
}
}
