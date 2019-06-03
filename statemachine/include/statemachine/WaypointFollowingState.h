#ifndef WAYPOINTFOLLOWINGSTATE_H
#define WAYPOINTFOLLOWINGSTATE_H

#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine/TeleoperationState.h>
#include <statemachine/StateInterface.h>
#include <statemachine_msgs/WaypointArray.h>
#include <statemachine_msgs/GetWaypoints.h>
#include <statemachine_msgs/SetWaypointFollowingMode.h>
#include <statemachine_msgs/WaypointVisited.h>
#include <std_srvs/Trigger.h>
#include <statemachine_msgs/SetNavigationGoal.h>

namespace statemachine {

/**
 * @class   WaypointFollowingState
 * @brief   State being active while
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

	statemachine_msgs::WaypointArray _waypoint_array;
	int _next_waypoint_position;

	ros::ServiceClient _get_waypoints_service;
	ros::ServiceClient _set_waypoint_following_mode_service;
	ros::ServiceClient _reset_waypoints_service;
	ros::ServiceClient _set_navigation_goal_service;
	ros::ServiceClient _waypoint_visited_service;

	void resetWaypoints();
	void abortWaypointFollowing();
	void getWaypoints();
	void setCurrentWaypointVisited();
};

}

#endif
