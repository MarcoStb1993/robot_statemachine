#ifndef IDLESTATE_H
#define IDLESTATE_H

#include <statemachine/BaseState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine/TeleoperationState.h>
#include <statemachine/WaypointFollowingState.h>
#include <statemachine/StateInterface.h>

namespace statemachine {

/**
 * @class   IdleState
 * @brief   State being active until all vital systems are running and ready.
 */
class IdleState: public BaseState {

public:

	/**
	 * Constructor
	 */
	IdleState();

	/**
	 * Destructor
	 */
	~IdleState();

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
