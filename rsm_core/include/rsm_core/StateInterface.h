#ifndef StateInterface_H_
#define StateInterface_H_

#include <rsm_msgs/OperationMode.h>
#include <rsm_msgs/SetNavigationGoal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <pluginlib/class_loader.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>

namespace rsm {

#define CALCULATEGOAL_STATE 1
#define NAVIGATION_STATE 2
#define MAPPING_STATE 3
#define ROUTINE_STATE 4

/**
 * @class StateInterface
 * @brief Handles the RSM transitions between the different state classes
 * 		  and holds a reference to the current and the upcoming state class
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
	boost::shared_ptr<rsm::BaseState> getPluginState(int plugin_type,
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
			boost::shared_ptr<rsm::BaseState> next_state);

private:
	ros::NodeHandle _nh;
	ros::Subscriber _operation_mode_sub;
	ros::Subscriber _simple_goal_sub;
	ros::ServiceServer _start_stop_exploration_service;
	ros::ServiceServer _start_stop_waypoint_following_service;
	ros::ServiceServer _stop_2d_nav_goal_service;
	ros::ServiceClient _set_navigation_goal_client;
	ros::ServiceServer _state_info_service;
	ros::Publisher _state_info_publisher;

	/**
	 * @brief Currently active state
	 */
	boost::shared_ptr<rsm::BaseState> _current_state;
	/**
	 * @brief Upcoming state
	 */
	boost::shared_ptr<rsm::BaseState> _next_state;
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
	pluginlib::ClassLoader<rsm::BaseState> _plugin_loader;

	/**
	 * Callback to receive the current operation mode and issue interrupts to the current state accordingly
	 * @param operation_mode Mode of operation
	 */
	void operationModeCallback(
			const rsm_msgs::OperationMode::ConstPtr& operation_mode);
	/**
	 * Callback receiving goals issued in RViz GUI with the 2D Nav Goal Tool and calling the respective
	 * interrupt in the current state
	 * @param goal Navigation goal being set in RViz GUI
	 */
	void simpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);
	bool startStopExplorationService(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	bool startStopWaypointFollowingService(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	bool stop2dNavGoal(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);
	bool stateInfoService(std_srvs::Trigger::Request &req,
			std_srvs::Trigger::Response &res);

};

}

#endif
