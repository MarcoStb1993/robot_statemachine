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
#include <std_srvs/Trigger.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <statemachine_msgs/SetNavigationGoal.h>
#include <statemachine_msgs/GetNavigationGoal.h>
#include <statemachine_msgs/AddFailedGoal.h>
#include <statemachine_msgs/GetFailedGoals.h>

#include <statemachine_msgs/GetRobotPose.h>
#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/PoseStamped.h>

namespace statemachine {

/**
 * @class ServiceProvider
 * @brief Establishes communication between the different states and the statemachine's
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
	ros::Subscriber _frontier_goals_subscriber;
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
	statemachine_msgs::WaypointArray _waypoint_array;
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
	 * List of all extracted frontier centers
	 */
	geometry_msgs::PoseArray _frontiers;
	/**
	 * Tolerance for comparing if the current goal is still in the list of frontiers
	 */
	double _exploration_goal_tolerance;
	/**
	 * Is navigation goal still a frontier
	 */
	bool _goal_obsolete;
	/**
	 * Mode of exploration (0=complete goal, 1=interrupt goal when frontier vanished)
	 */
	bool _exploration_mode;
	/**
	 * Is currently driving in reverse
	 */
	bool _reverse_mode_active;

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

	bool getRobotPose(statemachine_msgs::GetRobotPose::Request &req,
			statemachine_msgs::GetRobotPose::Response &res);

	bool getExplorationMode(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool setExplorationMode(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	void frontiersCallback(const geometry_msgs::PoseArray::ConstPtr& frontiers);
	/**
	 * Checks if the current navigation goal is still present as the center of a frontier
	 * @return Returns true if the current navigation goal is still the center of a frontier to be explored
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
