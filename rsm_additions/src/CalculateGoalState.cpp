#include <rsm_additions/CalculateGoalState.h>

namespace rsm {

CalculateGoalState::CalculateGoalState() {

}

CalculateGoalState::~CalculateGoalState() {
}

void CalculateGoalState::onSetup() {
	//initialize services, publisher and subscriber
	ros::NodeHandle nh("rsm");
	_frontiers_sub = nh.subscribe<geometry_msgs::PoseArray>("explorationGoals", 10,
			&CalculateGoalState::frontiersCallback, this);
	_get_failed_goals_service = nh.serviceClient<
			rsm_msgs::GetFailedGoals>("getFailedGoals");
	_set_navigation_goal_service = nh.serviceClient<
			rsm_msgs::SetNavigationGoal>("setNavigationGoal");
	_get_robot_pose_service = nh.serviceClient<rsm_msgs::GetRobotPose>(
			"getRobotPose");
	_idle_timer = nh.createTimer(ros::Duration(5.0),
			&CalculateGoalState::timerCallback, this, true);
	//initialize variables
	_name = "Calculate Goal";
	_frontiers_received = false;
}

void CalculateGoalState::onEntry() {
	//Request list of failed goals from Service Provider
	rsm_msgs::GetFailedGoals srv;
	if (_get_failed_goals_service.call(srv)) {
		_failed_goals = srv.response.failedGoals.poses;
	} else {
		ROS_ERROR("Failed to call Get Failed Goals service");
	}
}

void CalculateGoalState::onActive() {
	if (_frontiers_received) {
		//Calculate frontier center closest to the robot
		rsm_msgs::GetRobotPose srv;
		if (_get_robot_pose_service.call(srv)) {
			geometry_msgs::Pose current_pose = srv.response.pose;
			double min_distance = std::numeric_limits<double>::infinity();
			for (auto& pt : _frontier_points.poses) {
				if (differentFromFailedGoals(pt.position)) {
					double distance = pow(
							(current_pose.position.x - pt.position.x), 2)
							+ pow((current_pose.position.y - pt.position.y), 2);
					if (distance < min_distance) {
						min_distance = distance;
						_goal.position.x = pt.position.x;
						_goal.position.y = pt.position.y;
					}
				}
			}
			if (min_distance == std::numeric_limits<double>::infinity()) {
				ROS_ERROR(
						"Exploration stopped because there are no more reachable goals");
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
	rsm_msgs::SetNavigationGoal srv;
	srv.request.goal = _goal;
	srv.request.navigationMode = EXPLORATION;
	if (_set_navigation_goal_service.call(srv)) {
	} else {
		ROS_ERROR("Failed to call Set Navigation Goal service");
	}
}

void CalculateGoalState::onExplorationStart(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration running";
}

void CalculateGoalState::onExplorationStop(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration stopped";
	abortCalculateGoal();
}

void CalculateGoalState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void CalculateGoalState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void CalculateGoalState::onInterrupt(int interrupt) {
	switch (interrupt) {
	case EMERGENCY_STOP_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
		_interrupt_occured = true;
		break;
	case TELEOPERATION_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
		_interrupt_occured = true;
		break;
	case SIMPLE_GOAL_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(NAVIGATION_STATE));
		_interrupt_occured = true;
		break;
	}
}

void CalculateGoalState::frontiersCallback(
		const geometry_msgs::PoseArray::ConstPtr& frontiers) {
	_frontier_points = *frontiers;
	_frontiers_received = true;
}

bool CalculateGoalState::differentFromFailedGoals(geometry_msgs::Point point) {
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
	ROS_ERROR("Exploration stopped because no goal was selected in time");
	abortCalculateGoal();
}

void CalculateGoalState::abortCalculateGoal() {
	if (!_interrupt_occured) {
		_stateinterface->transitionToVolatileState(
				boost::make_shared<IdleState>());
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::CalculateGoalState,
		rsm::BaseState)
