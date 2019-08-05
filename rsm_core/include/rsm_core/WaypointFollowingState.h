#ifndef WAYPOINTFOLLOWINGSTATE_H
#define WAYPOINTFOLLOWINGSTATE_H

#include <rsm_msgs/WaypointArray.h>
#include <rsm_msgs/GetWaypoints.h>
#include <rsm_msgs/SetWaypointFollowingMode.h>
#include <rsm_msgs/WaypointVisited.h>
#include <std_srvs/Trigger.h>
#include <rsm_msgs/SetNavigationGoal.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/StateInterface.h>
#include <rsm_core/TeleoperationState.h>

namespace rsm {

/**
 * @class   WaypointFollowingState
 * @brief   State to decide which waypoint will be the next navigation goal while in waypoint following mode.
 */
class WaypointFollowingState: public BaseState {

public:

	/**
	 * Constructor
	 */
	WaypointFollowingState();

	/**
	 * Destructor
	 */
	~WaypointFollowingState();

	/**
	 * Called once when registered at StateInterface
	 */
	void onSetup();

	/**
	 * Called once when activated
	 */
	void onEntry();

	/**
	 * Process method (step-wise, never block this method)
	 */
	void onActive();

	/**
	 * Called once when left
	 */
	void onExit();

	/**
	 * Called when exploration was started manually
	 */
	void onExplorationStart(bool &success, std::string &message);

	/**
	 * Called when exploration was stopped manually
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

private:
	ros::ServiceClient _get_waypoints_service;
	ros::ServiceClient _set_waypoint_following_mode_service;
	ros::ServiceClient _reset_waypoints_service;
	ros::ServiceClient _set_navigation_goal_service;
	ros::ServiceClient _waypoint_visited_service;

	/**
	 * List of all waypoints
	 */
	rsm_msgs::WaypointArray _waypoint_array;
	/**
	 * Position of the next waypoint to navigate to in the list
	 */
	int _next_waypoint_position;

	/**
	 * Resets the status' of all waypoints and aborts waypoint following
	 */
	void resetWaypoints();
	/**
	 * Initiates transition to Idle State
	 */
	void abortWaypointFollowing();
	/**
	 * Requests all waypoints from Service Provider
	 */
	void getWaypoints();
	void setCurrentWaypointVisited();
};

}

#endif
