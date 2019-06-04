#ifndef NAVIGATIONSTATE_H
#define NAVIGATIONSTATE_H

#include <pluginlib/class_list_macros.h>
#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine_additions/ReversePathState.h>
#include <statemachine/StateInterface.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <statemachine_msgs/GetNavigationGoal.h>
#include <statemachine_msgs/AddFailedGoal.h>
#include <statemachine_msgs/WaypointVisited.h>
#include <statemachine_msgs/WaypointUnreachable.h>
#include <statemachine_msgs/GetRobotPose.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#define POSE_TOLERANCE 0.05
#define WAIT_TIME 3.0

namespace statemachine {

/**
 * @class   NavigationState
 * @brief   State being active until all vital systems are running and ready.
 */
class NavigationState: public BaseState {

public:

	/**
	 * Constructor
	 */
	NavigationState();

	/**
	 * Destructor
	 */
	~NavigationState();

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

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	MoveBaseClient _move_base_client;
	geometry_msgs::Pose _nav_goal;
	bool _goal_active;
	std::vector<geometry_msgs::Pose> _failed_goals;
	int _navigation_mode;
	int _waypoint_position;
	std::string _routine;
	bool _cmd_vel_recording;
	tf::Pose _last_pose;
	ros::Timer _idle_timer;
	int _comparison_counter;
	bool _exploration_mode;

	ros::NodeHandle _nh;
	ros::ServiceClient _get_navigation_goal_service;
	ros::ServiceClient _add_failed_goal_service;
	ros::ServiceClient _reset_failed_goals_service;
	ros::ServiceClient _waypoint_visited_service;
	ros::ServiceClient _waypoint_unreachable_service;
	ros::ServiceClient _get_robot_pose_service;
	ros::ServiceClient _get_exploration_mode;
	ros::Subscriber _get_goal_obsolete;

	/**
	 * @brief Callback for idle timer
	 * @param event
	 */
	void timerCallback(const ros::TimerEvent& event);
	void goalObsoleteCallback(const std_msgs::Bool::ConstPtr& msg);
	void abortNavigation();
	void comparePose();
};

}

#endif
