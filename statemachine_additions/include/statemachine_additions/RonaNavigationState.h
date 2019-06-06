#ifndef RONANAVIGATIONSTATE_H
#define RONANAVIGATIONSTATE_H

#include <pluginlib/class_list_macros.h>
#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine_additions/ReversePathState.h>
#include <statemachine/StateInterface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <statemachine_msgs/GetNavigationGoal.h>
#include <statemachine_msgs/AddFailedGoal.h>
#include <statemachine_msgs/WaypointVisited.h>
#include <statemachine_msgs/WaypointUnreachable.h>
#include <statemachine_msgs/GetRobotPose.h>
#include <statemachine_msgs/OperationMode.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_datatypes.h>
#include <rona_msgs/State.h>
#include <std_msgs/Bool.h>

#define POSE_TOLERANCE 0.05
#define WAIT_TIME 3.0

namespace statemachine {

/**
 * @class   RonaNavigationState
 * @brief   State being active when the robot should move to a previously set goal. First obtains the goal
 * 			and then tries to reach it using Rona navigation. State is exited when the goal was reached,
 * 			aborted or an interrupt	occured.
 */
class RonaNavigationState: public BaseState {

public:

	/**
	 * Constructor
	 */
	RonaNavigationState();

	/**
	 * Destructor
	 */
	~RonaNavigationState();

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
	/**
	 * Current state of the Sirona statemachine
	 */
	rona_msgs::State _sirona_state;
	/**
	 * Time at which the navigation goal was issued
	 */
	ros::Time _nav_start_time;

	ros::NodeHandle _nh;
	ros::ServiceClient _get_navigation_goal_service;
	ros::ServiceClient _add_failed_goal_service;
	ros::ServiceClient _reset_failed_goals_service;
	ros::ServiceClient _waypoint_visited_service;
	ros::ServiceClient _waypoint_unreachable_service;
	ros::ServiceClient _get_robot_pose_service;
	ros::ServiceClient _get_exploration_mode;
	ros::Subscriber _get_goal_obsolete;
	ros::Subscriber _sirona_state_subscriber;
	ros::Publisher _nav_goal_publisher;
	ros::ServiceClient _nav_stop_client;
	ros::Timer _idle_timer;

	void timerCallback(const ros::TimerEvent& event);
		/**
		 * Callback for checking if the current exploration goal is still viable or already obsolete,
		 * only checked for exploration mode 'interrupting"
		 * @param msg Is goal obsolete or not
		 */
		void goalObsoleteCallback(const std_msgs::Bool::ConstPtr& msg);
		/**
		 * Callback for receiving current state of the Sirona statemachine regarding navigation to the goal
		 * @param msg Current state
		 */
		void sironaStateCallback(const rona_msgs::State& msg);
		/**
		 * Callback for Operation mode to only enable the idle timer when operation mode is set to autonomous
		 * @param operation_mode Mode of operation (0=stopped, 1=autonomous, 2=teleoperation)
		 */
		void operationModeCallback(
				const statemachine_msgs::OperationMode::ConstPtr& operation_mode);
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
