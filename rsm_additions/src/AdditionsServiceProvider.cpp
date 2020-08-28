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
		_exploration_goals_publisher = nh.advertise<geometry_msgs::PoseArray>(
				"explorationGoals", 1);
		_exploration_mode_subscriber = nh.subscribe("explorationMode", 1,
				&AdditionsServiceProvider::explorationModeCallback, this);
		_failed_goals_publisher = nh.advertise<geometry_msgs::PoseArray>(
				"failedGoals", 1);
		_exploration_goal_subscriber = nh.subscribe("explorationGoalStatus", 1,
				&AdditionsServiceProvider::explorationGoalCallback, this);
		_get_navigation_goal_service = nh.serviceClient<
				rsm_msgs::GetNavigationGoal>("getNavigationGoal");
	} else {
		_calculate_goal_plugin_used = false;
	}

	std::string mapping_plugin;
	private_nh.param<std::string>("mapping_plugin", mapping_plugin, "");
	if (mapping_plugin.compare("rsm::RealsenseMappingState") == 0) {
		_reset_realsense_position_serivce = nh.advertiseService(
				"resetRealsensePosition",
				&AdditionsServiceProvider::resetRealsensePosition, this);
		_realsense_joint_controller = _nh.advertise<std_msgs::Float64>(
				"realsense_controller/command", 1, true);
	}

	_exploration_mode = 0;
	_goal_obsolete = false;
}

AdditionsServiceProvider::~AdditionsServiceProvider() {

}

void AdditionsServiceProvider::publishTopics() {
	if (_calculate_goal_plugin_used) {
		publishExplorationGoals();
		publishFailedGoals();
	}
	if (_exploration_mode) {
		publishGoalObsolete();
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
	_exploration_goals_publisher.publish(_exploration_goals);
}

void AdditionsServiceProvider::publishFailedGoals() {
	_failed_goals.header.frame_id = "map";
	_failed_goals.header.stamp = ros::Time::now();
	_failed_goals_publisher.publish(_failed_goals);
}

void AdditionsServiceProvider::publishGoalObsolete() {
	std_msgs::Bool msg;
	msg.data = _goal_obsolete;
	_goal_obsolete_publisher.publish(msg);
}

void AdditionsServiceProvider::explorationGoalCallback(
		const rsm_msgs::GoalStatus::ConstPtr& goal_status) {
	if (goal_status->goal_status == rsm_msgs::GoalStatus::REACHED) {
		_failed_goals.poses.clear();
	} else if (goal_status->goal_status == rsm_msgs::GoalStatus::FAILED) {
		_failed_goals.poses.push_back(goal_status->goal);
	}
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
		_goal_obsolete = true;
	} else {
		_goal_obsolete = false;
	}
}

void AdditionsServiceProvider::explorationModeCallback(
		const std_msgs::Bool::ConstPtr& exploration_mode) {
	_exploration_mode = exploration_mode->data;
	if (_exploration_mode) {
		ros::NodeHandle nh("rsm");
		_goal_obsolete_publisher = nh.advertise<std_msgs::Bool>("goalObsolete",
				1);
	} else {
		_goal_obsolete_publisher.shutdown();
	}
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

bool AdditionsServiceProvider::resetRealsensePosition(
		std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	std_msgs::Float64 realsense_command;
	realsense_command.data = 0.0;	//center position of realsense camera
	_realsense_joint_controller.publish(realsense_command);
	return true;
}
}
