#ifndef BOOTSTATE_H
#define BOOTSTATE_H

#include <std_srvs/SetBool.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/StateInterface.h>

namespace rsm {

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
	ros::ServiceClient _bootupClient;

};

}

#endif
