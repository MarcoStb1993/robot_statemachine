#ifndef CALCULATEGOALSTATE_H
#define CALCULATEGOALSTATE_H

#include <pluginlib/class_list_macros.h>
#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine/TeleoperationState.h>
#include <statemachine/StateInterface.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PoseArray.h>
#include <statemachine_msgs/SetNavigationGoal.h>
#include <statemachine_msgs/GetFailedGoals.h>
#include <statemachine_msgs/GetRobotPose.h>
#include <tf/transform_listener.h>

namespace statemachine {

/**
 * @class   CalculateGoalState
 * @brief   State being active until all vital systems are running and ready.
 */
class CalculateGoalState: public BaseState {

public:

	/**
	 * @brief Constructor
	 */
	CalculateGoalState();

	/**
	 * @brief Destructor
	 */
	~CalculateGoalState();

	/**
	 * @brief Called once when registered at StateInterface
	 */
	void onSetup();

	/**
	 * @brief Called once when activated
	 */
	void onEntry();

	/**
	 * @brief Process method (step-wise, never block this method)
	 */
	void onActive();

	/**
	 * @brief Called once when left
	 */
	void onExit();

	/**
	 * Called when exploration was started manually
	 */
	void onExplorationStart(bool &success, std::string &message);

	/**
	 * @brief Called when exploration was stopped manually
	 */
	void onExplorationStop(bool &success, std::string &message);

	/**
	 * Called when waypoint following was started/paused manually
	 */
	void onWaypointFollowingStart(bool &success, std::string &message);

	/**
	 * Called when waypoint following was stopped manually
	 */
	void onWaypointFollowingStop(bool &success, std::string &message);

	/**
	 * @brief Called when an operation mode interrupt was received
	 * @param interrupt Kind of interrupt (0=EmergencyStop, 1=TeleoperationInterupt)
	 */
	void onInterrupt(int interrupt);

	/**
	 * @brief Called when new frontiers are received
	 * @param PointCloud2 with all frontier points
	 */
	void frontiersCallback(
			const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& frontiers);

	/**
	 * @brief Checks if a point is different from a previously failed goals including a small tolerance
	 * @param Point that is checked against previously failed goals
	 * @return Returns if the given point is different from the previously failed goals
	 */
	bool differentFromFailedGoals(pcl::PointXYZ point);

private:

	ros::NodeHandle _nh;
	ros::Subscriber _frontiers_sub;
	std::vector<geometry_msgs::Pose> _failed_goals;
	geometry_msgs::Pose _goal;
	std::vector<pcl::PointXYZ> _frontier_points;
	bool _frontiers_received;

	ros::ServiceClient _get_failed_goals_service;
	ros::ServiceClient _set_navigation_goal_service;
	ros::ServiceClient _get_robot_pose_service;

	/**
	 * @brief Timer for callback after receiving no new teleoperation commands
	 */
	ros::Timer _idle_timer;

	/**
	 * @brief Callback for idle timer
	 * @param event
	 */
	void timerCallback(const ros::TimerEvent& event);
	void abortCalculateGoal();
};

}

#endif
