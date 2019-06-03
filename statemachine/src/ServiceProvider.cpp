#include <statemachine/ServiceProvider.h>

namespace statemachine {

ServiceProvider::ServiceProvider() {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("robot_frame", _robot_frame, "/base_link");
	private_nh.param<std::string>("autonomy_cmd_vel_topic",
			_autonomy_cmd_vel_topic, "/cmd_vel");
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

	_start_stop_cmd_vel_recording_service = nh.advertiseService(
			"startStopCmdVelRecording",
			&ServiceProvider::startStopCmdVelRecording, this);
	_get_cmd_vel_recording_service = nh.advertiseService("getCmdVelRecording",
			&ServiceProvider::getCmdVelRecording, this);
	_request_reverse_path_usage_service = nh.advertiseService(
			"requestReversePathUsage",
			&ServiceProvider::requestReversePathUsage, this);
	_reset_reverse_path_usage_service = nh.advertiseService(
			"resetReversePathUsage", &ServiceProvider::resetReversePathUsage,
			this);
	_reset_cmd_vel_recording_service = nh.advertiseService(
			"resetCmdVelRecording", &ServiceProvider::resetCmdVelRecording,
			this);

	_set_reverse_mode_service = nh.advertiseService("setReverseMode",
			&ServiceProvider::setReverseMode, this);
	_get_reverse_mode_service = nh.advertiseService("getReverseMode",
			&ServiceProvider::getReverseMode, this);
	_reverse_mode_publisher = nh.advertise<std_msgs::Bool>("reverseMode", 10);

	_get_robot_pose_service = nh.advertiseService("getRobotPose",
			&ServiceProvider::getRobotPose, this);

	_waypoint_array.header.seq = 0;
	_waypoint_array.header.stamp = ros::Time::now();
	_waypoint_array.header.frame_id = "map";
	_waypoint_array.mode = 0;
	_waypoint_array.reverse = false;
	_waypoint_array.waypoints_size = 0;

	_waypoint_following = false;
	_waypoint_position = -1;

	double controller_frequency;
	_nh.param("/move_base/controller_frequency", controller_frequency, 20.0);
	_msg_buffer_size = controller_frequency * 2; //2 seconds recorded cmd vel messages
	_cmd_vel_msgs.set_capacity(_msg_buffer_size);
	_reverse_path_used = false;

	_reverse_mode_active = false;

}

ServiceProvider::~ServiceProvider() {

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

bool ServiceProvider::startStopCmdVelRecording(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data) {
		ros::NodeHandle nh;
		_cmd_vel_subscriber = nh.subscribe(_autonomy_cmd_vel_topic, 10,
				&ServiceProvider::cmdVelCallback, this);
		res.message = "Started Cmd Vel recording";
	} else {
		_cmd_vel_subscriber.shutdown();
		res.message = "Stopped Cmd Vel recording";
	}
	res.success = 1;
	return true;
}

bool ServiceProvider::getCmdVelRecording(
		statemachine_msgs::GetCmdVelRecording::Request &req,
		statemachine_msgs::GetCmdVelRecording::Response &res) {
	std::vector<geometry_msgs::Twist> cmd_vel_msgs;
	for (int i = 0; i < _cmd_vel_msgs.size(); i++) {
		cmd_vel_msgs.push_back(_cmd_vel_msgs[i]);
	}
	res.cmdVelMsgs = cmd_vel_msgs;
	res.waypointFollowing = _waypoint_following;
	return true;
}

bool ServiceProvider::resetCmdVelRecording(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	_cmd_vel_msgs.clear();
	res.success = 1;
	res.message = "Cmd Vel recording reset";
	return true;
}

bool ServiceProvider::requestReversePathUsage(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
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

bool ServiceProvider::resetReversePathUsage(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
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

void ServiceProvider::cmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& msg) {
	if (msg->linear.x != 0 || msg->linear.y != 0 || msg->linear.z != 0
			|| msg->angular.x != 0 || msg->angular.y != 0
			|| msg->angular.z != 0) {
		_cmd_vel_msgs.push_back(*msg);
	}
}

bool ServiceProvider::setReverseMode(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (req.data) {
		if (_reverse_mode_active) {
			res.success = 0;
			res.message = "Already in reverse mode";
		} else {
			_reverse_mode_active = true;
			res.success = 1;
			res.message = "Set to reverse mode";
		}
	} else {
		if (_reverse_mode_active) {
			res.success = 1;
			res.message = "Set to forward mode";
			_reverse_mode_active = false;
		} else {
			res.success = 0;
			res.message = "Already in forward mode";
		}
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

void ServiceProvider::publishTopics() {
	publishWaypoints();
	publishReverseMode();
}

}
