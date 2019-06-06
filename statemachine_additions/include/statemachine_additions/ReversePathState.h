#ifndef REVERSEPATHSTATE_H_
#define REVERSEPATHSTATE_H_

#include <statemachine/BaseState.h>
#include <statemachine/IdleState.h>
#include <statemachine/EmergencyStopState.h>
#include <statemachine/TeleoperationState.h>
#include <statemachine/StateInterface.h>

#include <statemachine_msgs/GetCmdVelRecording.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

namespace statemachine {

/**
 * @class   ReversePathState
 * @brief   State to replay the last few seconds of cmd vel topic in reversed order with negated speeds
 * 			to move the same way back when robot is stuck.
 */
class ReversePathState: public BaseState {
public:
	ReversePathState();
	~ReversePathState();
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
	ros::ServiceClient _get_cmd_vel_recording_service;
	ros::ServiceClient _reset_cmd_vel_recording_service;
	ros::Publisher _cmd_vel_publisher;
	ros::Timer _cmd_vel_replay_timer;

	std::string _autonomy_cmd_vel_topic;

	/**
	 * Mode of navigation (Exploration=0, Waypoint following=1 and Simple Goal=2)
	 */
	int _navigation_mode;
	/**
	 * If negated cmd vel is still replayed
	 */
	bool _cmd_vel_replaying;
	/**
	 * List of cmd vel messages to be negated and replayed
	 */
	std::vector<geometry_msgs::Twist> _cmd_vel_msgs;
	/**
	 * Current cmd vel message to be replayed
	 */
	int _current_cmd_vel_msg;

	bool startStopCmdVelReversedReplay(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	/**
	 * Callback for timer to publish the cmd vel messages in reverse order with the same frequency as the
	 * original publisher. Initiates state transition when playback ended
	 * @param event
	 */
	void timerCallback(const ros::TimerEvent& event);
	/**
	 * Publish a reversed cmd vel message
	 */
	void publishReverseCmdVelMsg();
	/**
	 * Initiates transition to Idle State
	 */
	void abortNavigation();
};

} /* namespace statemachine */

#endif /* REVERSEPATHSTATE_H_ */
