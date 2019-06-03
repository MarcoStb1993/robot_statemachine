#ifndef BOOTSTATE_H
#define BOOTSTATE_H

#include <statemachine/BaseState.h>
#include <statemachine/StateInterface.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <std_srvs/SetBool.h>

namespace statemachine {

/**
 * @class   BootState
 * @brief   State being active until all vital systems are running and ready.
 */
class BootState: public BaseState {

public:

	/**
	 * @brief Constructor
	 */
	BootState();

	/**
	 * @brief Destructor
	 */
	~BootState();

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
	void onWaypointFollowingStartStop(bool &success, std::string &message);

	/**
	 * @brief Called when an operation mode interrupt was received
	 * @param interrupt Kind of interrupt (0=EmergencyStop, 1=TeleoperationInterupt)
	 */
	void onInterrupt(int interrupt);

private:

	/**
	 * @brief Service client for bootup service
	 */
	ros::ServiceClient _bootupClient;

};

}

#endif
