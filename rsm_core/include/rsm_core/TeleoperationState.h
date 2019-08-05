#ifndef TELEOPERATIONSTATE_H
#define TELEOPERATIONSTATE_H

#include <rsm_core/BaseState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/StateInterface.h>

namespace rsm {

/**
 * @class   TeleoperationState
 * @brief   State being active while the robot is teleoperated
 */
class TeleoperationState: public BaseState {

public:

	/**
	 * Constructor
	 */
	TeleoperationState();

	/**
	 * Destructor
	 */
	~TeleoperationState();

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

};

}

#endif
