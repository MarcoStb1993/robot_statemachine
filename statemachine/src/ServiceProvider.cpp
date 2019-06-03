#include <statemachine/ServiceProvider.h>

namespace statemachine {

ServiceProvider::ServiceProvider() {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame, "/base_link");
	private_nh.param<std::vector<std::string>>("waypoint_routines",
			_waypoint_routines, std::vector<std::string>());

	ros::NodeHandle nh("statemachine");
	_add_waypoint_service = nh.advertiseService("addWaypoint",
			&ServiceProvider::addWaypoint, this);
	_get_waypoints_service = nh.advertiseService("getWaypoints",
			&ServiceProvider::getWaypoints, this);
	_move_waypoint_service = nh.advertiseService("moveWaypoint",
			&ServiceProvider::moveWaypoint, this);
	_remove_waypoint_service = nh.advertiseService("removeWaypoint",
			&ServiceProvider::removeWaypoint, this);
	_waypoint_visited_service = nh.advertiseService("waypointVisited",
			&ServiceProvider::waypointVisited, this);
	_waypoint_unreachable_service = nh.advertiseService("waypointUnreachable",
			&ServiceProvider::waypointUnreachable, this);
	_reset_waypoints_service = nh.advertiseService("resetWaypoints",
			&ServiceProvider::resetWaypoints, this);
	_set_waypoint_following_mode_service = nh.advertiseService(
			"setWaypointFollowingMode",
			&ServiceProvider::setWaypointFollowingMode, this);
	_set_waypoint_routine_service = nh.advertiseService("setWaypointRoutine",
			&ServiceProvider::setWaypointRoutine, this);
	_get_waypoint_routines_service = nh.advertiseService("getWaypointRoutines",
			&ServiceProvider::getWaypointRoutines, this);
	_waypoints_publisher = nh.advertise<statemachine_msgs::WaypointArray>(
			"waypoints", 10);

	_set_navigation_goal_service = nh.advertiseService("setNavigationGoal",
			&ServiceProvider::setNavigationGoal, this);
	_get_navigation_goal_service = nh.advertiseService("getNavigationGoal",
			&ServiceProvider::getNavigationGoal, this);
	_add_failed_goal_service = nh.advertiseService("addFailedGoal",
			&ServiceProvider::addFailedGoal, this);
	_get_failed_goals_service = nh.advertiseService("getFailedGoals",
			&ServiceProvider::getFailedGoals, this);
	_reset_failed_goals_service = nh.advertiseService("resetFailedGoals",
			&ServiceProvider::resetFailedGoals, this);

	_get_robot_pose_service = nh.advertiseService("getRobotPose",
			&ServiceProvider::getRobotPose, this);

	_set_exploration_mode_service = nh.advertiseService("setExplorationMode",
			&ServiceProvider::setExplorationMode, this);
	_get_exploration_mode_service = nh.advertiseService("getExplorationMode",
			&ServiceProvider::getExplorationMode, this);
	_frontier_goals_subscriber = nh.subscribe("frontiers", 1,
			&ServiceProvider::frontiersCallback, this);
	_goal_obsolete_publisher = nh.advertise<std_msgs::Bool>("goalObsolete", 1);
	_exploration_mode_publisher = nh.advertise<std_msgs::Bool>(
			"explorationMode", 1);

	_waypoint_array.header.seq = 0;
	_waypoint_array.header.stamp = ros::Time::now();
	_waypoint_array.header.frame_id = "map";
	_waypoint_array.mode = 0;
	_waypoint_array.reverse = false;
	_waypoint_array.waypoints_size = 0;

	_waypoint_following = false;
	_waypoint_position = -1;

	_goal_obsolete = false;
	_exploration_mode = 0;
}

ServiceProvider::~ServiceProvider() {

}

void ServiceProvider::publishTopics() {
	publishWaypoints();
	publishGoalObsolete();
	publishExplorationModes();
}

bool ServiceProvider::addWaypoint(statemachine_msgs::AddWaypoint::Request &req,
		statemachine_msgs::AddWaypoint::Response &res) {
	if (req.position >= 0 && req.position < _waypoint_array.waypoints_size) {
		_waypoint_array.waypoints.insert(
				_waypoint_array.waypoints.begin() + req.position, req.waypoint);
		res.message = "Added waypoint successfully";
	} else {
		_waypoint_array.waypoints.push_back(req.waypoint);
		res.message = "Added waypoint successfully at the end";
	}
	_waypoint_array.waypoints_size++;
	res.success = 1;
	return true;
}

bool ServiceProvider::getWaypoints(
		statemachine_msgs::GetWaypoints::Request &req,
		statemachine_msgs::GetWaypoints::Response &res) {
	res.waypointArray = _waypoint_array;
	return true;
}

bool ServiceProvider::moveWaypoint(
		statemachine_msgs::MoveWaypoint::Request &req,
		statemachine_msgs::MoveWaypoint::Response &res) {
	if (req.position >= 0 && req.position < _waypoint_array.waypoints_size) {
		_waypoint_array.waypoints[req.position].pose = req.pose;
		res.success = 1;
		res.message = "Moved waypoint successfully";
	} else {
		res.success = 0;
		res.message = "Position out of waypoint array range";
	}
	return true;
}

bool ServiceProvider::removeWaypoint(
		statemachine_msgs::RemoveWaypoint::Request &req,
		statemachine_msgs::RemoveWaypoint::Response &res) {
	if (req.position >= 0 && req.position < _waypoint_array.waypoints_size) {
		_waypoint_array.waypoints.erase(
				_waypoint_array.waypoints.begin() + req.position);
		_waypoint_array.waypoints_size--;
		res.success = 1;
		res.message = "Removed waypoint successfully";
	} else {
		res.success = 0;
		res.message = "Position out of waypoint array range";
	}
	return true;
}

bool ServiceProvider::waypointVisited(
		statemachine_msgs::WaypointVisited::Request &req,
		statemachine_msgs::WaypointVisited::Response &res) {
	if (req.position >= 0 && req.position < _waypoint_array.waypoints_size) {
		_waypoint_array.waypoints[req.position].visited = true;
		res.success = 1;
		res.message = "Waypoint visited";
	} else {
		res.success = 0;
		res.message = "Position out of waypoint array range";
	}
	return true;
}

bool ServiceProvider::waypointUnreachable(
		statemachine_msgs::WaypointUnreachable::Request &req,
		statemachine_msgs::WaypointUnreachable::Response &res) {
	if (req.position >= 0 && req.position < _waypoint_array.waypoints_size) {
		_waypoint_array.waypoints[req.position].unreachable = true;
		res.success = 1;
		res.message = "Waypoint unreachable";
	} else {
		res.success = 0;
		res.message = "Position out of waypoint array range";
	}
	return true;
}

bool ServiceProvider::resetWaypoints(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	for (auto& it : _waypoint_array.waypoints) {
		it.visited = false;
		it.unreachable = false;
	}
	res.success = 1;
	res.message = "Waypoints reset";
	return true;
}

bool ServiceProvider::setWaypointFollowingMode(
		statemachine_msgs::SetWaypointFollowingMode::Request &req,
		statemachine_msgs::SetWaypointFollowingMode::Response &res) {
	_waypoint_array.mode = req.mode;
	_waypoint_array.reverse = req.reverse;
	res.success = 1;
	res.message = "Waypoint Following mode changed";
	return true;
}

bool ServiceProvider::setWaypointRoutine(
		statemachine_msgs::SetWaypointRoutine::Request &req,
		statemachine_msgs::SetWaypointRoutine::Response &res) {
	if (req.position >= 0 && req.position < _waypoint_array.waypoints_size) {
		_waypoint_array.waypoints[req.position].routine = req.routine;
		res.success = 1;
		res.message = "Waypoint routine set";
	} else {
		res.success = 0;
		res.message = "Position out of waypoint array range";
	}
	return true;
}

bool ServiceProvider::getWaypointRoutines(
		statemachine_msgs::GetWaypointRoutines::Request &req,
		statemachine_msgs::GetWaypointRoutines::Response &res) {
	res.waypointRoutines = _waypoint_routines;
	return true;
}

void ServiceProvider::publishWaypoints() {
	_waypoints_publisher.publish(_waypoint_array);
}

bool ServiceProvider::setNavigationGoal(
		statemachine_msgs::SetNavigationGoal::Request &req,
		statemachine_msgs::SetNavigationGoal::Response &res) {
	_navigation_goal = req.goal;
	_waypoint_following = req.waypointFollowing;
	_waypoint_position = req.waypointPosition;
	_routine = req.routine;
	res.success = 1;
	res.message = "Navigation goal set";
	return true;
}

bool ServiceProvider::getNavigationGoal(
		statemachine_msgs::GetNavigationGoal::Request &req,
		statemachine_msgs::GetNavigationGoal::Response &res) {
	res.goal = _navigation_goal;
	res.failedGoals = _failed_goals;
	res.waypointFollowing = _waypoint_following;
	res.waypointPosition = _waypoint_position;
	res.routine = _routine;
	return true;
}

bool ServiceProvider::addFailedGoal(
		statemachine_msgs::AddFailedGoal::Request &req,
		statemachine_msgs::AddFailedGoal::Response &res) {
	_failed_goals.poses.push_back(req.failedGoal);
	res.success = 1;
	res.message = "Failed goal added";
	return true;
}

bool ServiceProvider::getFailedGoals(
		statemachine_msgs::GetFailedGoals::Request &req,
		statemachine_msgs::GetFailedGoals::Response &res) {
	res.failedGoals = _failed_goals;
	return true;
}

bool ServiceProvider::resetFailedGoals(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	_failed_goals.poses.clear();
	res.success = 1;
	res.message = "Failed goals reset";
	return true;
}
bool ServiceProvider::getRobotPose(
		statemachine_msgs::GetRobotPose::Request &req,
		statemachine_msgs::GetRobotPose::Response &res) {
	_transform_listener.lookupTransform("/map", _robot_frame, ros::Time(0),
			_transform);
	geometry_msgs::Pose pose;
	pose.position.x = _transform.getOrigin().x();
	pose.position.y = _transform.getOrigin().y();
	pose.position.z = _transform.getOrigin().z();
	pose.orientation.x = _transform.getRotation().x();
	pose.orientation.y = _transform.getRotation().y();
	pose.orientation.z = _transform.getRotation().z();
	pose.orientation.w = _transform.getRotation().w();
	res.pose = pose;
	return true;
}

bool ServiceProvider::getExplorationMode(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	res.success = _exploration_mode;
	res.message = "Exploration mode returned";
	return true;
}

bool ServiceProvider::setExplorationMode(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	_exploration_mode = req.data;
	res.success = 1;
	res.message = "Exploration mode set";
	return true;
}

void ServiceProvider::frontiersCallback(
		const geometry_msgs::PoseArray::ConstPtr& frontiers) {
	_frontiers = *frontiers;
}

bool ServiceProvider::navGoalIncludedInFrontiers() {
	double tolerance = 0.05;
	for (auto iterator : _frontiers.poses) {
		double x_dif = abs(_navigation_goal.position.x - iterator.position.x);
		double y_dif = abs(_navigation_goal.position.y - iterator.position.y);
		if (x_dif <= tolerance && y_dif <= tolerance) {
			return true;
		}
	}
	return false;
}

void ServiceProvider::publishGoalObsolete() {
	if (_exploration_mode == 1 && !navGoalIncludedInFrontiers()) {
		_goal_obsolete = true;
	} else {
		_goal_obsolete = false;
	}
	std_msgs::Bool msg;
	msg.data = _goal_obsolete;
	_goal_obsolete_publisher.publish(msg);
}

void ServiceProvider::publishExplorationModes() {
	std_msgs::Bool msg;
	msg.data = _exploration_mode;
	_exploration_mode_publisher.publish(msg);
}

}
