#ifndef REVERSINGROUTINESTATE_H
#define REVERSINGROUTINESTATE_H

#include <pluginlib/class_list_macros.h>
#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine/WaypointFollowingState.h>
#include <statemachine/TeleoperationState.h>
#include <statemachine/StateInterface.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>

namespace statemachine {

/**
 * @class   ReversingRoutineState
 * @brief   State being active until all vital systems are running and ready.
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
	ros::ServiceClient _set_rona_reverse_on;
	ros::ServiceClient _set_rona_reverse_off;

	bool _reverse_mode_active;
};

}

#endif
