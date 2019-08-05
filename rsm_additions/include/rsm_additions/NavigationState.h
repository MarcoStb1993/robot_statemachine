#ifndef NAVIGATIONSTATE_H
#define NAVIGATIONSTATE_H

#include <pluginlib/class_list_macros.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/StateInterface.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rsm_msgs/GetNavigationGoal.h>
#include <rsm_msgs/AddFailedGoal.h>
#include <rsm_msgs/WaypointVisited.h>
#include <rsm_msgs/WaypointUnreachable.h>
#include <rsm_msgs/GetRobotPose.h>
#include <rsm_msgs/OperationMode.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Bool.h>

#define POSE_TOLERANCE 0.05

namespace rsm {

/**
 * @class   NavigationState
 * @brief   State being active when the robot should move to a previously set goal. First obtains the goal
 * 			and then tries to reach it using the Move Base package. State is exited when the goal was reached,
 * 			aborted or an interrupt	occured.
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
	/**
	 * Navigation goal to reach
	 */
	geometry_msgs::Pose _nav_goal;
	/**
	 * Is goal currently active or does it need to be set first
	 */
	bool _goal_active;
	/**
	 * List of previously failed goals
	 */
	std::vector<geometry_msgs::Pose> _failed_goals;
	/**
	 * Mode of navigation (Exploration=0, Waypoint following=1 and Simple Goal=2)
	 */
	int _navigation_mode;
	/**
	 * Position of waypoint in waypoint array
	 */
	int _waypoint_position;
	/**
	 * Routine to be executed when reaching waypoint
	 */
	std::string _routine;
	/**
	 * Last pose of the robot
	 */
	tf::Pose _last_pose;
	/**
	 * Counter for comparing last with current pose only every 5th call
	 */
	int _comparison_counter;
	/**
	 * Mode of exploration (0=complete goal, 1=interrupt goal when frontier vanished)
	 */
	bool _exploration_mode;
	/**
	 * Is robot driving in reverse or not
	 */
	bool _reverse_mode;
	/**
	 * Currently active mode of operation (0=stopped, 1=autonomous, 2=teleoperation)
	 */
	int _operation_mode;

	typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
	boost::shared_ptr<MoveBaseClient> _move_base_client;

	ros::NodeHandle _nh;
	ros::ServiceClient _get_navigation_goal_service;
	ros::ServiceClient _add_failed_goal_service;
	ros::ServiceClient _reset_failed_goals_service;
	ros::ServiceClient _waypoint_visited_service;
	ros::ServiceClient _waypoint_unreachable_service;
	ros::ServiceClient _get_robot_pose_service;
	ros::ServiceClient _get_exploration_mode_service;
	ros::ServiceClient _get_reverse_mode_service;
	ros::Subscriber _get_goal_obsolete;
	ros::Subscriber _reverse_mode_subscriber;
	ros::Subscriber _exploration_mode_subscriber;
	ros::Timer _idle_timer;

	/**
	 * @brief Callback for idle timer
	 * @param event
	 */
	void timerCallback(const ros::TimerEvent& event);
	/**
	 * Callback for checking if the current exploration goal is still viable or already obsolete,
	 * only checked for exploration mode 'interrupting"
	 * @param msg Is goal obsolete or not
	 */
	void goalObsoleteCallback(const std_msgs::Bool::ConstPtr& msg);
	/**
	 * Callback for checking if reverse mode changed and if it did, reset SimpleActionClient to
	 * interface the Move Base node for reverse movement
	 * @param reverse_mode Is reverse mode active or not
	 */
	void reverseModeCallback(const std_msgs::Bool::ConstPtr& reverse_mode);
	/**
	 * Callback for Operation mode to only enable the idle timer when operation mode is set to autonomous
	 * @param operation_mode Mode of operation (0=stopped, 1=autonomous, 2=teleoperation)
	 */
	void operationModeCallback(
			const rsm_msgs::OperationMode::ConstPtr& operation_mode);
	/**
	 * Initiate transition to Idle State
	 */
	void abortNavigation();
	/**
	 * Check if the robot's pose changed and reset idle timer if it did, restart it if not
	 */
	void comparePose();
};

}

#endif
