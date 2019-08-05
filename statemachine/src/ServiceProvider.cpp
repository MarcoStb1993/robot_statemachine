#include <rsm_core/ServiceProvider.h>

namespace rsm {

ServiceProvider::ServiceProvider() {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame, "/base_link");
	private_nh.param<std::vector<std::string>>("waypoint_routines",
			_waypoint_routines, std::vector<std::string>());
	private_nh.param<double>("exploration_goal_tolerance",
			_exploration_goal_tolerance, 0.05);

	ros::NodeHandle nh("rsm");
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
	_waypoints_publisher = nh.advertise<rsm_msgs::WaypointArray>(
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
	_exploration_mode_publisher = nh.advertise<std_msgs::Bool>(
			"explorationMode", 1);

	_set_reverse_mode_service = nh.advertiseService("setReverseMode",
			&ServiceProvider::setReverseMode, this);
	_get_reverse_mode_service = nh.advertiseService("getReverseMode",
			&ServiceProvider::getReverseMode, this);
	_set_navigation_to_reverse_client = nh.serviceClient<std_srvs::SetBool>(
			"setNavigationToReverse");
	_reverse_mode_publisher = nh.advertise<std_msgs::Bool>("reverseMode", 10);

	_waypoint_array.header.seq = 0;
	_waypoint_array.header.stamp = ros::Time::now();
	_waypoint_array.header.frame_id = "map";
	_waypoint_array.mode = 0;
	_waypoint_array.reverse = false;
	_waypoint_array.waypoints_size = 0;

	_navigation_mode = -1;
	_waypoint_position = -1;

	_goal_obsolete = false;
	_exploration_mode = 0;

	_reverse_mode_active = false;
}

ServiceProvider::~ServiceProvider() {

}

void ServiceProvider::publishTopics() {
	publishWaypoints();
	publishExplorationModes();
	publishReverseMode();
	if (_exploration_mode) {
		publishGoalObsolete();

	}
}

bool ServiceProvider::addWaypoint(rsm_msgs::AddWaypoint::Request &req,
		rsm_msgs::AddWaypoint::Response &res) {
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
		rsm_msgs::GetWaypoints::Request &req,
		rsm_msgs::GetWaypoints::Response &res) {
	res.waypointArray = _waypoint_array;
	return true;
}

bool ServiceProvider::moveWaypoint(
		rsm_msgs::MoveWaypoint::Request &req,
		rsm_msgs::MoveWaypoint::Response &res) {
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
		rsm_msgs::RemoveWaypoint::Request &req,
		rsm_msgs::RemoveWaypoint::Response &res) {
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
		rsm_msgs::WaypointVisited::Request &req,
		rsm_msgs::WaypointVisited::Response &res) {
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
		rsm_msgs::WaypointUnreachable::Request &req,
		rsm_msgs::WaypointUnreachable::Response &res) {
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
		rsm_msgs::SetWaypointFollowingMode::Request &req,
		rsm_msgs::SetWaypointFollowingMode::Response &res) {
	_waypoint_array.mode = req.mode;
	_waypoint_array.reverse = req.reverse;
	res.success = 1;
	res.message = "Waypoint Following mode changed";
	return true;
}

bool ServiceProvider::setWaypointRoutine(
		rsm_msgs::SetWaypointRoutine::Request &req,
		rsm_msgs::SetWaypointRoutine::Response &res) {
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
		rsm_msgs::GetWaypointRoutines::Request &req,
		rsm_msgs::GetWaypointRoutines::Response &res) {
	res.waypointRoutines = _waypoint_routines;
	return true;
}

void ServiceProvider::publishWaypoints() {
	_waypoints_publisher.publish(_waypoint_array);
}

bool ServiceProvider::setNavigationGoal(
		rsm_msgs::SetNavigationGoal::Request &req,
		rsm_msgs::SetNavigationGoal::Response &res) {
	_navigation_goal = req.goal;
	_navigation_mode = req.navigationMode;
	_waypoint_position = req.waypointPosition;
	_routine = req.routine;
	res.success = 1;
	res.message = "Navigation goal set";
	return true;
}

bool ServiceProvider::getNavigationGoal(
		rsm_msgs::GetNavigationGoal::Request &req,
		rsm_msgs::GetNavigationGoal::Response &res) {
	res.goal = _navigation_goal;
	res.failedGoals = _failed_goals;
	res.navigationMode = _navigation_mode;
	res.waypointPosition = _waypoint_position;
	res.routine = _routine;
	return true;
}

bool ServiceProvider::addFailedGoal(
		rsm_msgs::AddFailedGoal::Request &req,
		rsm_msgs::AddFailedGoal::Response &res) {
	_failed_goals.poses.push_back(req.failedGoal);
	res.success = 1;
	res.message = "Failed goal added";
	return true;
}

bool ServiceProvider::getFailedGoals(
		rsm_msgs::GetFailedGoals::Request &req,
		rsm_msgs::GetFailedGoals::Response &res) {
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
		rsm_msgs::GetRobotPose::Request &req,
		rsm_msgs::GetRobotPose::Response &res) {
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
	if (_exploration_mode) {
		ros::NodeHandle nh("rsm");
		_goal_obsolete_publisher = nh.advertise<std_msgs::Bool>("goalObsolete",
				1);
		_exploration_goals_subscriber = nh.subscribe("explorationGoals", 1,
				&ServiceProvider::explorationGoalCallback, this);
	} else {
		_goal_obsolete_publisher.shutdown();
		_exploration_goals_subscriber.shutdown();
	}
	res.success = 1;
	res.message = "Exploration mode set";
	return true;
}

void ServiceProvider::explorationGoalCallback(
		const geometry_msgs::PoseArray::ConstPtr& exploration_goals) {
	_exploration_goals = *exploration_goals;
}

bool ServiceProvider::navGoalIncludedInFrontiers() {
	for (auto iterator : _exploration_goals.poses) {
		double x_dif = abs(_navigation_goal.position.x - iterator.position.x);
		double y_dif = abs(_navigation_goal.position.y - iterator.position.y);
		double z_dif = abs(_navigation_goal.position.z - iterator.position.z);
		if (x_dif <= _exploration_goal_tolerance
				&& y_dif <= _exploration_goal_tolerance
				&& z_dif <= _exploration_goal_tolerance) {
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

bool ServiceProvider::setReverseMode(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data != _reverse_mode_active) {
		_reverse_mode_active = req.data;
		std_srvs::SetBool srv;
		srv.request.data = _reverse_mode_active;
		if (_set_navigation_to_reverse_client.call(srv)) {
			if (srv.response.success) {
				res.success = 1;
				res.message = "Mode set";
			} else {
				res.success = 0;
				res.message = "Mode not set";
			}
		} else {
			ROS_ERROR("Unable to call Set Navigation To Reverse service");
			res.success = 0;
			res.message = "Unable to set reverse mode";
		}
	} else {
		res.success = 0;
		res.message = "Already in requested mode";
	}
	return true;
}

bool ServiceProvider::getReverseMode(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	if (_reverse_mode_active) {
		res.success = true;
		res.message = "Reverse mode active";
	} else {
		res.success = false;
		res.message = "Forward mode active";
	}
	return true;
}

void ServiceProvider::publishReverseMode() {
	std_msgs::Bool msg;
	msg.data = _reverse_mode_active;
	_reverse_mode_publisher.publish(msg);
}

}
