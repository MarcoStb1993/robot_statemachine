#include <statemachine_additions/CalculateGoalState.h>

namespace statemachine {

CalculateGoalState::CalculateGoalState() {
	ROS_INFO("CalculateGoalState constructed");
	_name = "Calculate Goal";
}

CalculateGoalState::~CalculateGoalState() {
	ROS_INFO("CalculateGoalState destructed");
}

void CalculateGoalState::onSetup() {
	ROS_INFO("CalculateGoalState setup");
	_frontiers_sub = _nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(
			"/explore_server/explore_costmap/explore_boundary/frontiers", 10,
			&CalculateGoalState::frontiersCallback, this);
	_frontiers_received = false;
	ros::NodeHandle nh("statemachine");
	_get_failed_goals_service = nh.serviceClient<
			statemachine_msgs::GetFailedGoals>("getFailedGoals");
	_set_navigation_goal_service = nh.serviceClient<
			statemachine_msgs::SetNavigationGoal>("setNavigationGoal");
	_get_robot_pose_service = nh.serviceClient<statemachine_msgs::GetRobotPose>(
			"getRobotPose");

	_idle_timer = nh.createTimer(ros::Duration(5.0),
			&CalculateGoalState::timerCallback, this, true);
}

void CalculateGoalState::onEntry() {
	ROS_INFO("CalculateGoalState entered");
	statemachine_msgs::GetFailedGoals srv;
	if (_get_failed_goals_service.call(srv)) {
		_failed_goals = srv.response.failedGoals.poses;
	} else {
		ROS_ERROR("Failed to call Get Failed Goals service");
	}
}

void CalculateGoalState::onActive() {
	//ROS_INFO("CalculateGoalState active");
	if (_frontiers_received) {
		statemachine_msgs::GetRobotPose srv;
		if (_get_robot_pose_service.call(srv)) {
			geometry_msgs::Pose current_pose = srv.response.pose;
			double min_distance = std::numeric_limits<double>::infinity();
			BOOST_FOREACH (pcl::PointXYZ& pt, _frontier_points) {
				if (differentFromFailedGoals(pt)) {
					double distance = pow((current_pose.position.x - pt.x), 2)
							+ pow((current_pose.position.y - pt.y), 2);
					if (distance < min_distance) {
						min_distance = distance;
						_goal.position.x = pt.x;
						_goal.position.y = pt.y;
					}
				}
			}
			if (min_distance == std::numeric_limits<double>::infinity()) {
				ROS_INFO("No more reachable goals, Exploration stopped");
				abortCalculateGoal();
			} else {
				double yaw = atan2(_goal.position.y - current_pose.position.y,
						_goal.position.x - current_pose.position.x);
				_goal.orientation = tf::createQuaternionMsgFromYaw(yaw);
				_frontiers_received = false;
				if (!_interrupt_occured) {
					_stateinterface->transitionToVolatileState(
							_stateinterface->getPluginState(NAVIGATION_STATE));
				}
			}

		} else {
			ROS_ERROR("Failed to call Get Robot Pose service");
			abortCalculateGoal();
		}
	}
}

void CalculateGoalState::onExit() {
	ROS_INFO("CalculateGoalState exited");
	statemachine_msgs::SetNavigationGoal srv;
	srv.request.goal = _goal;
	srv.request.waypointFollowing = false;
	if (_set_navigation_goal_service.call(srv)) {
	} else {
		ROS_ERROR("Failed to call Set Navigation Goal service");
	}
}

void CalculateGoalState::onExplorationStart(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Start called in CalculateGoalState");
	success = true;
	message = "Exploration running";
}

void CalculateGoalState::onExplorationStop(bool &success,
		std::string &message) {
	ROS_INFO("Exploration Stop called in CalculateGoalState");
	success = true;
	message = "Exploration stopped";
	abortCalculateGoal();
}

void CalculateGoalState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following start/pause called in CalculateGoalState");
	success = false;
	message = "Exploration running";
}

void CalculateGoalState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	ROS_INFO("Waypoint following stop called in CalculateGoalState");
	success = false;
	message = "Exploration running";
}

void CalculateGoalState::onInterrupt(int interrupt) {
	if (interrupt == EMERGENCY_STOP_INTERRUPT) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
	} else {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
	}
	_interrupt_occured = true;
}

void CalculateGoalState::frontiersCallback(
		const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& frontiers) {
	ROS_INFO("Frontiers received in CalculateGoalState");
	_frontier_points.clear();
	BOOST_FOREACH (const pcl::PointXYZ& pt, frontiers->points) {
		_frontier_points.push_back(pt);
	}
	_frontiers_received = true;
}

bool CalculateGoalState::differentFromFailedGoals(pcl::PointXYZ point) {
	double tolerance = 0.05;
	for (auto iterator : _failed_goals) {
		double x_dif = abs(point.x - iterator.position.x);
		double y_dif = abs(point.y - iterator.position.y);
		if (x_dif <= tolerance && y_dif <= tolerance) {
			return false;
		}
	}
	return true;
}

void CalculateGoalState::timerCallback(const ros::TimerEvent& event) {
	ROS_ERROR("Calculate Goal State received no goals");
	abortCalculateGoal();
}

void CalculateGoalState::abortCalculateGoal() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(statemachine::CalculateGoalState,
		statemachine::BaseState)
