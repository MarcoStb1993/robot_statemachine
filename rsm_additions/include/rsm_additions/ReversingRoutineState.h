#ifndef REVERSINGROUTINESTATE_H
#define REVERSINGROUTINESTATE_H

#include <pluginlib/class_list_macros.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/WaypointFollowingState.h>
#include <rsm_core/TeleoperationState.h>
#include <rsm_core/StateInterface.h>
#include <rsm_msgs/GoalCompleted.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

namespace rsm {

/**
 * @class   ReversingRoutineState
 * @brief   Routine state that toggles the reverse mode.
 */
class ReversingRoutineState: public BaseState {

public:

	/**
	 * Constructor
	 */
	ReversingRoutineState();

	/**
	 * Destructor
	 */
	~ReversingRoutineState();

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
	 * @param interrupt Kind of interrupt (1=EmergencyStop, 2=TeleoperationInterupt)
	 */
	void onInterrupt(int interrupt);

private:
	ros::NodeHandle _nh;
	ros::ServiceClient _set_reverse_moving_service;
	ros::ServiceClient _get_reverse_moving_service;
	ros::ServiceClient _navigation_goal_completed_service;

	/**
	 * Is the robot currently driving in reverse
	 */
	bool _reverse_mode_active;
	/**
	 * Was the routine at the waypoint goal successful or not
	 */
	int _navigation_completed_status;
};

}

#endif
