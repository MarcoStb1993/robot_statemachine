#ifndef StateInterface_H_
#define StateInterface_H_

#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine_msgs/OperationMode.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <pluginlib/class_loader.h>
#include <map>

namespace statemachine {

#define CALCULATEGOAL_STATE 1
#define NAVIGATION_STATE 2
#define MAPPING_STATE 3
#define ROUTINE_STATE 4

/**
 * StateInterface class handles the statemachine transitions between the different state classes
 * and holds a reference to the current and the upcoming state class
 */
class StateInterface {

public:

	/**
	 * Constructor
	 */
	StateInterface();

	/**
	 * Destructor
	 */
	virtual ~StateInterface();

	/**
	 * @brief Returns a plugin-state corresponding to the given type
	 * @param Plugin state type
	 * @param Routine plugin name
	 * @return Pointer to plugin state
	 */
	boost::shared_ptr<statemachine::BaseState> getPluginState(int plugin_type,
			std::string routine = "");

	/**
	 * Awake StateInterface to do his job
	 */
	void awake();

	/**
	 * Activate new state instance. Instance is deleted with next transition.
	 * @param nextState
	 */
	void transitionToVolatileState(
			boost::shared_ptr<statemachine::BaseState> next_state);

private:
	ros::NodeHandle _nh;
	ros::Subscriber _operation_mode_sub;
	ros::ServiceServer _start_stop_exploration_service;
	ros::ServiceServer _start_stop_waypoint_following_service;
	ros::Publisher _state_info_publisher;

	/**
	 * @brief Currently active state
	 */
	boost::shared_ptr<statemachine::BaseState> _current_state;
	/**
	 * @brief Upcoming state
	 */
	boost::shared_ptr<statemachine::BaseState> _next_state;
	/**
	 * Interrupt happened before
	 */
	bool _on_interrupt;

	/**
	 * @brief Calculate goal state plugin name for plugin loader
	 */
	std::string _calculate_goal_plugin;
	/**
	 * @brief Navigation state plugin name for plugin loader
	 */
	std::string _navigation_plugin;
	/**
	 * @brief Mapping state plugin name for plugin loader
	 */
	std::string _mapping_plugin;
	/**
	 * @brief Plugin loader for state plugins
	 */
	pluginlib::ClassLoader<statemachine::BaseState> _plugin_loader;

	void operationModeCallback(
			const statemachine_msgs::OperationMode::ConstPtr& operation_mode);
	bool startStopExplorationService(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	bool startStopWaypointFollowingService(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);

};

}

#endif
