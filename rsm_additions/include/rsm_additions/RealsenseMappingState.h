#ifndef MAPPINGSTATE_H
#define MAPPINGSTATE_H

#include <pluginlib/class_list_macros.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/TeleoperationState.h>
#include <rsm_core/StateInterface.h>
#include <rsm_msgs/GoalCompleted.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

#define MOVE_LEFT 0
#define MOVE_RIGHT 1
#define MOVE_TO_CENTER 2

#define REALSENSE_LEFT_LIMIT 1.30
#define REALSENSE_RIGHT_LIMIT -1.30
#define REALSENSE_CENTER_POSITION 0.0

#define POS_TOLERANCE 0.05

namespace rsm {

/**
 * @class   RealsenseMappingState
 * @brief   Dummy state for mapping at a reached goal during exploration. Only initiates transition to
 * 			CalculateGoalState.
 */
class RealsenseMappingState: public BaseState {

public:

	/**
	 * Constructor
	 */
	RealsenseMappingState();

	/**
	 * Destructor
	 */
	~RealsenseMappingState();

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

	ros::NodeHandle _nh;
	ros::Subscriber _joint_states_subscriber;
	ros::Publisher _realsense_joint_controller;
	ros::ServiceClient _reset_realsense_position_client;
	ros::ServiceClient _navigation_goal_completed_service;

	/**
	 * Current state of swiveling the Realsense camera from left to right and back (0: to left, 1: left to right: 2: back to center)
	 */
	int _swivel_state;
	/**
	 * If the current state reached it's goal already
	 */
	bool _position_reached;
	/**
	 * Move command sent to realsense controller
	 */
	bool _message_send;
	/**
	 * Was the mapping at the exploration goal successful or not
	 */
	int _navigation_completed_status;

	/**
	 * Callback for joint states to check if camera reached the desired position
	 * @param joint_state joint state message
	 */
	void jointStateCallback(sensor_msgs::JointState::ConstPtr joint_state);
};

}

#endif
