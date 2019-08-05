#include <ros/ros.h>

#include <rsm_msgs/Waypoint.h>
#include <rsm_msgs/WaypointArray.h>
#include <rsm_msgs/AddWaypoint.h>
#include <rsm_msgs/GetWaypoints.h>
#include <rsm_msgs/MoveWaypoint.h>
#include <rsm_msgs/RemoveWaypoint.h>
#include <rsm_msgs/WaypointVisited.h>
#include <rsm_msgs/WaypointUnreachable.h>
#include <rsm_msgs/SetWaypointFollowingMode.h>
#include <rsm_msgs/SetWaypointRoutine.h>
#include <rsm_msgs/GetWaypointRoutines.h>
#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <rsm_msgs/SetNavigationGoal.h>
#include <rsm_msgs/GetNavigationGoal.h>
#include <rsm_msgs/AddFailedGoal.h>
#include <rsm_msgs/GetFailedGoals.h>

#include <rsm_msgs/GetRobotPose.h>
#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>

namespace rsm {

/**
 * @class ServiceProvider
 * @brief Establishes communication between the different states and the RSM's
 * 		  periphery including the GUI. It offers services and publishes topics based on the variables that need
 * 		  to be saved during state transitions.
 */
class ServiceProvider {

public:
	/**
	 * Constructor
	 */
	ServiceProvider();
	/**
	 * Destructor
	 */
	~ServiceProvider();
	/**
	 * Publish all topics (waypoints, goal obsolete, exploration and reverse mode)
	 */
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

	ros::ServiceServer _get_robot_pose_service;
	tf::TransformListener _transform_listener;

	ros::ServiceServer _set_exploration_mode_service;
	ros::ServiceServer _get_exploration_mode_service;
	ros::Subscriber _exploration_goals_subscriber;
	ros::Publisher _goal_obsolete_publisher;
	ros::Publisher _exploration_mode_publisher;

	ros::ServiceServer _set_reverse_mode_service;
	ros::ServiceServer _get_reverse_mode_service;
	ros::ServiceClient _set_navigation_to_reverse_client;
	ros::Publisher _reverse_mode_publisher;

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
	rsm_msgs::WaypointArray _waypoint_array;
	/**
	 * Mode of navigation (Exploration=0, Waypoint following=1 and Simple Goal=2)
	 */
	int _navigation_mode;
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

	/**
	 * @brief ROS transform from map to robot frame
	 */
	tf::StampedTransform _transform;
	;
	/**
	 * @brief Robot frame id
	 */
	std::string _robot_frame;
	/**
	 * List of all extracted exploration goals
	 */
	geometry_msgs::PoseArray _exploration_goals;
	/**
	 * Tolerance for comparing if the current goal is still in the list of exploration goals
	 */
	double _exploration_goal_tolerance;
	/**
	 * Is navigation goal still an exploration goal
	 */
	bool _goal_obsolete;
	/**
	 * Mode of exploration (0=complete goal, 1=interrupt goal when exploration goals vanished)
	 */
	bool _exploration_mode;
	/**
	 * Is currently driving in reverse
	 */
	bool _reverse_mode_active;

	bool addWaypoint(rsm_msgs::AddWaypoint::Request &req,
			rsm_msgs::AddWaypoint::Response &res);
	bool getWaypoints(rsm_msgs::GetWaypoints::Request &req,
			rsm_msgs::GetWaypoints::Response &res);
	bool moveWaypoint(rsm_msgs::MoveWaypoint::Request &req,
			rsm_msgs::MoveWaypoint::Response &res);
	bool removeWaypoint(rsm_msgs::RemoveWaypoint::Request &req,
			rsm_msgs::RemoveWaypoint::Response &res);
	bool waypointVisited(rsm_msgs::WaypointVisited::Request &req,
			rsm_msgs::WaypointVisited::Response &res);
	bool waypointUnreachable(
			rsm_msgs::WaypointUnreachable::Request &req,
			rsm_msgs::WaypointUnreachable::Response &res);
	bool resetWaypoints(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool setWaypointFollowingMode(
			rsm_msgs::SetWaypointFollowingMode::Request &req,
			rsm_msgs::SetWaypointFollowingMode::Response &res);
	bool setWaypointRoutine(rsm_msgs::SetWaypointRoutine::Request &req,
			rsm_msgs::SetWaypointRoutine::Response &res);
	bool getWaypointRoutines(
			rsm_msgs::GetWaypointRoutines::Request &req,
			rsm_msgs::GetWaypointRoutines::Response &res);
	void publishWaypoints();

	bool setNavigationGoal(rsm_msgs::SetNavigationGoal::Request &req,
			rsm_msgs::SetNavigationGoal::Response &res);
	bool getNavigationGoal(rsm_msgs::GetNavigationGoal::Request &req,
			rsm_msgs::GetNavigationGoal::Response &res);
	bool addFailedGoal(rsm_msgs::AddFailedGoal::Request &req,
			rsm_msgs::AddFailedGoal::Response &res);
	bool getFailedGoals(rsm_msgs::GetFailedGoals::Request &req,
			rsm_msgs::GetFailedGoals::Response &res);
	bool resetFailedGoals(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);

	bool getRobotPose(rsm_msgs::GetRobotPose::Request &req,
			rsm_msgs::GetRobotPose::Response &res);

	bool getExplorationMode(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool setExplorationMode(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	void explorationGoalCallback(const geometry_msgs::PoseArray::ConstPtr& exploration_goals);
	/**
	 * Checks if the current navigation goal is still present as an exploration goal
	 * @return Returns true if the current navigation goal is still an exploration goal to be explored
	 */
	bool navGoalIncludedInFrontiers();
	void publishGoalObsolete();
	void publishExplorationModes();

	bool setReverseMode(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	bool getReverseMode(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	void publishReverseMode();
};

}
