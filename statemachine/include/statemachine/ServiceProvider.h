#include <ros/ros.h>

#include <statemachine_msgs/Waypoint.h>
#include <statemachine_msgs/WaypointArray.h>
#include <statemachine_msgs/AddWaypoint.h>
#include <statemachine_msgs/GetWaypoints.h>
#include <statemachine_msgs/MoveWaypoint.h>
#include <statemachine_msgs/RemoveWaypoint.h>
#include <statemachine_msgs/WaypointVisited.h>
#include <statemachine_msgs/WaypointUnreachable.h>
#include <statemachine_msgs/SetWaypointFollowingMode.h>
#include <statemachine_msgs/SetWaypointRoutine.h>
#include <statemachine_msgs/GetWaypointRoutines.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <statemachine_msgs/SetNavigationGoal.h>
#include <statemachine_msgs/GetNavigationGoal.h>
#include <statemachine_msgs/AddFailedGoal.h>
#include <statemachine_msgs/GetFailedGoals.h>
#include <statemachine_msgs/GetCmdVelRecording.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Bool.h>
#include <boost/circular_buffer.hpp>

#include <statemachine_msgs/GetRobotPose.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

namespace statemachine {

class ServiceProvider {

public:
	ServiceProvider();
	~ServiceProvider();
	void publishTopics();

private:
	ros::NodeHandle _nh;
	ros::ServiceServer _add_waypoint_service;
	ros::ServiceServer _get_waypoints_service;
	ros::ServiceServer _move_waypoint_service;
	ros::ServiceServer _remove_waypoint_service;
	ros::ServiceServer _waypoint_visited_service;
	ros::ServiceServer _waypoint_unreachable_service;
	ros::ServiceServer _reset_waypoints_service;
	ros::ServiceServer _set_waypoint_following_mode_service;
	ros::ServiceServer _set_waypoint_routine_service;
	ros::ServiceServer _get_waypoint_routines_service;
	ros::Publisher _waypoints_publisher;

	ros::ServiceServer _set_navigation_goal_service;
	ros::ServiceServer _get_navigation_goal_service;
	ros::ServiceServer _add_failed_goal_service;
	ros::ServiceServer _get_failed_goals_service;
	ros::ServiceServer _reset_failed_goals_service;

	ros::ServiceServer _start_stop_cmd_vel_recording_service;
	ros::ServiceServer _get_cmd_vel_recording_service;
	ros::ServiceServer _reset_cmd_vel_recording_service;
	ros::ServiceServer _request_reverse_path_usage_service;
	ros::ServiceServer _reset_reverse_path_usage_service;
	ros::Subscriber _cmd_vel_subscriber;

	ros::ServiceServer _set_reverse_mode_service;
	ros::ServiceServer _get_reverse_mode_service;
	ros::Publisher _reverse_mode_publisher;

	ros::ServiceServer _get_robot_pose_service;
	tf::TransformListener _transform_listener;

	/**
	 * Current navigation goal
	 */
	geometry_msgs::Pose _navigation_goal;
	/**
	 * List of previously failed goals
	 */
	geometry_msgs::PoseArray _failed_goals;
	/**
	 * List of all waypoints
	 */
	statemachine_msgs::WaypointArray _waypoint_array;
	/**
	 * Is waypoint following active
	 */
	bool _waypoint_following;
	/**
	 * Routine to be executed when reaching the current waypoint
	 */
	std::string _routine;
	/**
	 * Position of the current waypoint in the waypoint list
	 */
	int _waypoint_position;
	/**
	 *
	 * List of all available waypoint routines
	 */
	std::vector<std::string> _waypoint_routines;

	bool _reverse_path_used;
	boost::circular_buffer<geometry_msgs::Twist> _cmd_vel_msgs;
	int _msg_buffer_size;
	std::string _autonomy_cmd_vel_topic;

	/**
	 * Is currently driving in reverse
	 */
	bool _reverse_mode_active;
	/**
	 * @brief ROS transform from map to robot frame
	 */
	tf::StampedTransform _transform;
	;
	/**
	 * @brief Robot frame id
	 */
	std::string _robot_frame;

	bool addWaypoint(statemachine_msgs::AddWaypoint::Request &req,
			statemachine_msgs::AddWaypoint::Response &res);
	bool getWaypoints(statemachine_msgs::GetWaypoints::Request &req,
			statemachine_msgs::GetWaypoints::Response &res);
	bool moveWaypoint(statemachine_msgs::MoveWaypoint::Request &req,
			statemachine_msgs::MoveWaypoint::Response &res);
	bool removeWaypoint(statemachine_msgs::RemoveWaypoint::Request &req,
			statemachine_msgs::RemoveWaypoint::Response &res);
	bool waypointVisited(statemachine_msgs::WaypointVisited::Request &req,
			statemachine_msgs::WaypointVisited::Response &res);
	bool waypointUnreachable(
			statemachine_msgs::WaypointUnreachable::Request &req,
			statemachine_msgs::WaypointUnreachable::Response &res);
	bool resetWaypoints(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool setWaypointFollowingMode(
			statemachine_msgs::SetWaypointFollowingMode::Request &req,
			statemachine_msgs::SetWaypointFollowingMode::Response &res);
	bool setWaypointRoutine(statemachine_msgs::SetWaypointRoutine::Request &req,
			statemachine_msgs::SetWaypointRoutine::Response &res);
	bool getWaypointRoutines(
			statemachine_msgs::GetWaypointRoutines::Request &req,
			statemachine_msgs::GetWaypointRoutines::Response &res);
	void publishWaypoints();

	bool setNavigationGoal(statemachine_msgs::SetNavigationGoal::Request &req,
			statemachine_msgs::SetNavigationGoal::Response &res);
	bool getNavigationGoal(statemachine_msgs::GetNavigationGoal::Request &req,
			statemachine_msgs::GetNavigationGoal::Response &res);
	bool addFailedGoal(statemachine_msgs::AddFailedGoal::Request &req,
			statemachine_msgs::AddFailedGoal::Response &res);
	bool getFailedGoals(statemachine_msgs::GetFailedGoals::Request &req,
			statemachine_msgs::GetFailedGoals::Response &res);
	bool resetFailedGoals(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);

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

	bool getRobotPose(statemachine_msgs::GetRobotPose::Request &req,
			statemachine_msgs::GetRobotPose::Response &res);
};

}
