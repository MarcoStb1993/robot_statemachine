#include <rsm_additions/AdditionsServiceProvider.h>

namespace rsm {

AdditionsServiceProvider::AdditionsServiceProvider() :
		as(NULL) {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("autonomy_cmd_vel_topic",
			_autonomy_cmd_vel_topic, "/autonomy/cmd_vel");

	ros::NodeHandle nh("rsm");

	std::string navigation_plugin;
	private_nh.param<std::string>("navigation_plugin", navigation_plugin, "");
	if (navigation_plugin.compare("rsm::NavigationState") == 0) {
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

	std::string calculate_goal_plugin;
	private_nh.param<std::string>("calculate_goal_plugin",
			calculate_goal_plugin, "");
	if (calculate_goal_plugin.compare("rsm::CalculateGoalState") == 0) {
		_calculate_goal_plugin_used = true;
		private_nh.param<double>("exploration_goal_tolerance",
				_exploration_goal_tolerance, 0.05);
		as = new MoveBaseActionServer(_nh, "frontier_move_base",
				boost::bind(&AdditionsServiceProvider::navigationGoalCallback,
						this, _1), false);
		as->start();
		frontiers_marker_array_subscriber = _nh.subscribe("explore/frontiers",
				1, &AdditionsServiceProvider::frontierCallback, this);
		exploration_goals_publisher = nh.advertise<geometry_msgs::PoseArray>(
				"explorationGoals", 1);
		_exploration_mode_subscriber = nh.subscribe("explorationMode", 1,
				&AdditionsServiceProvider::explorationModeCallback, this);
		_set_goal_obsolete_service = nh.serviceClient<std_srvs::Trigger>(
				"setGoalObsolete");
		_add_failed_goal_service = nh.advertiseService("addFailedGoal",
				&AdditionsServiceProvider::addFailedGoal, this);
		_get_failed_goals_service = nh.advertiseService("getFailedGoals",
				&AdditionsServiceProvider::getFailedGoals, this);
		_reset_failed_goals_service = nh.advertiseService("resetFailedGoals",
				&AdditionsServiceProvider::resetFailedGoals, this);
		_exploration_goal_completed_service = nh.advertiseService(
				"explorationGoalCompleted",
				&AdditionsServiceProvider::explorationGoalCompleted, this);
		_get_navigation_goal_service = nh.serviceClient<
				rsm_msgs::GetNavigationGoal>("getNavigationGoal");
	} else {
		_calculate_goal_plugin_used = false;
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

	_exploration_mode = 0;
}

AdditionsServiceProvider::~AdditionsServiceProvider() {

}

void AdditionsServiceProvider::publishTopics() {
	if (_calculate_goal_plugin_used) {
		publishExplorationGoals();
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

bool AdditionsServiceProvider::addFailedGoal(
		rsm_msgs::AddFailedGoal::Request &req,
		rsm_msgs::AddFailedGoal::Response &res) {
	_failed_goals.poses.push_back(req.failedGoal);
	res.success = 1;
	res.message = "Failed goal added";
	return true;
}

bool AdditionsServiceProvider::getFailedGoals(
		rsm_msgs::GetFailedGoals::Request &req,
		rsm_msgs::GetFailedGoals::Response &res) {
	res.failedGoals = _failed_goals;
	return true;
}

bool AdditionsServiceProvider::resetFailedGoals(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	_failed_goals.poses.clear();
	res.success = 1;
	res.message = "Failed goals reset";
	return true;
}

bool AdditionsServiceProvider::explorationGoalCompleted(
		rsm_msgs::ExplorationGoalCompleted::Request &req,
		rsm_msgs::ExplorationGoalCompleted::Response &res) {
	if (req.goal_reached) {
		_failed_goals.poses.clear();
		res.message = "Failed goals cleared";
	} else {
		_failed_goals.poses.push_back(req.goal);
		res.message = "Failed goal added";
	}
	res.success = 1;
	return true;
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
	if (_exploration_mode && !navGoalIncludedInFrontiers()) {
		std_srvs::Trigger srv;
		if (!_set_goal_obsolete_service.call(srv)) {
			ROS_ERROR("Failed to call Set Goal Obsolete service");
		}
	}
}

void AdditionsServiceProvider::explorationModeCallback(
		const std_msgs::Bool::ConstPtr& exploration_mode) {
	_exploration_mode = exploration_mode->data;
}

bool AdditionsServiceProvider::navGoalIncludedInFrontiers() {
	ROS_INFO_STREAM("Goal tolerance: " << _exploration_goal_tolerance);
	rsm_msgs::GetNavigationGoal srv;
	if (_get_navigation_goal_service.call(srv)) {
		geometry_msgs::Pose navigation_goal = srv.response.goal;
		for (auto iterator : _exploration_goals.poses) {
			double x_dif = abs(
					navigation_goal.position.x - iterator.position.x);
			double y_dif = abs(
					navigation_goal.position.y - iterator.position.y);
			double z_dif = abs(
					navigation_goal.position.z - iterator.position.z);
			if (x_dif <= _exploration_goal_tolerance
					&& y_dif <= _exploration_goal_tolerance
					&& z_dif <= _exploration_goal_tolerance) {
				return true;
			}
		}
	} else {
		ROS_ERROR("Failed to call Get Navigation Goal service");
	}
	return false;
}

bool AdditionsServiceProvider::resetKinectPosition(
		std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	std_msgs::Float64 kinetic_command;
	kinetic_command.data = 0.0;	//center position of kinect camera
	_kinetic_joint_controller.publish(kinetic_command);
	return true;
}
}
