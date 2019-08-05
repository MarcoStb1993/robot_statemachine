#ifndef BASESTATE_H
#define BASESTATE_H

#include <stddef.h>
#include <ros/ros.h>
#include <ros/console.h>

namespace rsm {

#define INTERRUPT_END 0
#define EMERGENCY_STOP_INTERRUPT 1
#define TELEOPERATION_INTERRUPT 2
#define SIMPLE_GOAL_INTERRUPT 3
#define SIMPLE_GOAL_STOP_INTERRUPT 4

#define EXPLORATION 0
#define WAYPOINT_FOLLOWING 1
#define SIMPLE_GOAL 2

class StateInterface;

/**
 * @class Base State
 * @brief Abstract base state with virtual methods for implementation of states
 */
class BaseState {
public:

	/**
	 * Constructor
	 */
	BaseState();

	/**
	 * Default destructor
	 */
	virtual ~BaseState();

	/**
	 * Set state interface of state. This is called by the state interface at state transitions.
	 * @param StateInterface Instance
	 */
	void setStateInterface(StateInterface* stateinterface);

	/**
	 * Get state interface of state. Default is NULL, when no state interface is assigned.
	 * @return pointer to StateInterface instance.
	 */
	StateInterface* getStateInterface();

	/**
	 * Called once when registered at StateInterface
	 */
	virtual void onSetup() {
	}
	;

	/**
	 * Called once when activated
	 */
	virtual void onEntry() {
	}
	;

	/**
	 * Called while active
	 */
	virtual void onActive() = 0;

	/**
	 * Called once when left
	 */
	virtual void onExit() {
	}
	;

	/**
	 * Called when exploration was started manually
	 */
	virtual void onExplorationStart(bool &success,
			std::string &message) {
	}
	;

	/**
	 * Called when exploration was stopped manually
	 */
	virtual void onExplorationStop(bool &success, std::string &message) {
	}
	;

	/**
	 * Called when waypoint following was started/paused manually
	 */
	virtual void onWaypointFollowingStart(bool &success, std::string &message) {
	}
	;

	/**
	 * Called when waypoint following was stopped manually
	 */
	virtual void onWaypointFollowingStop(bool &success, std::string &message) {
	}
	;

	/**
	 * @brief Called when an operation mode interrupt was received
	 * @param interrupt Kind of interrupt (1=EmergencyStop, 2=TeleoperationInterupt)
	 */
	virtual void onInterrupt(int interrupt) {
	}
	;

	/**
	 * Getter for the state's name
	 * @return Name of the state
	 */
	std::string getName();

protected:

	/**
	 * @brief Pointer to State Interface handling all state transitions
	 */
	StateInterface* _stateinterface;
	/**
	 * @brief Shows if an interupt occured
	 */
	bool _interrupt_occured;
	/**
	 * @brief Name of the state
	 */
	std::string _name;
};

}

#endif
