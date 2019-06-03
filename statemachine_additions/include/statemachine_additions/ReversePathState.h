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
 * @brief   State being active until all vital systems are running and ready.
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

	bool _waypoint_following;
	bool _cmd_vel_replaying;
	std::vector<geometry_msgs::Twist> _cmd_vel_msgs;
	int _current_cmd_vel_msg;
	ros::Timer _cmd_vel_replay_timer;
	std::string _autonomy_cmd_vel_topic;

	bool startStopCmdVelReversedReplay(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	void timerCallback(const ros::TimerEvent& event);
	void publishReverseCmdVelMsg();
	void abortNavigation();
};

} /* namespace statemachine */

#endif /* REVERSEPATHSTATE_H_ */
