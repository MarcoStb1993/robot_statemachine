#ifndef ROBOTCONTROLMUX_H_
#define ROBOTCONTROLMUX_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <statemachine_msgs/OperationMode.h>
#include <statemachine_msgs/SetOperationMode.h>

#define STOPPED_OPERATION 0
#define AUTONOMOUS_OPERATION 1
#define TELEOPERATION 2

namespace statemachine {

/**
 * Class RobotControlMux handles all command velocities and only allows the robot to move
 * at all if no emergency stop is set and only move autonomous if autonomous operation is
 * active
 */
class RobotControlMux {
public:
	/**
	 * Constructor
	 */
	RobotControlMux();
	/**
	 * Destructor
	 */
	~RobotControlMux();

	void publishTopics();

private:
	ros::NodeHandle _nh;
	ros::ServiceServer _set_operation_mode_service;
	ros::Subscriber _teleoperation_cmd_vel_sub;
	ros::Subscriber _autonomy_cmd_vel_sub;
	ros::Publisher _cmd_vel_pub;
	ros::Publisher _operation_mode_pub;

	ros::Timer _teleoperation_idle_timer;

	std::string _teleoperation_cmd_vel_topic;
	std::string _autonomy_operation_cmd_vel_topic;
	std::string _cmd_vel_topic;

	/**
	 * Time until teleoperation mode will stopped when no new message is received
	 */
	double _teleoperation_idle_timer_duration;
	/**
	 * Currently active mode of operation (0=stopped, 1=autonomous, 2=teleoperation)
	 */
	int _operation_mode;
	/**
	 * Is emergency stop currently activated
	 */
	bool _emergency_stop_active;
	/**
	 * Last received command velocity from autonomy
	 */
	geometry_msgs::Twist _autonomy_cmd_vel;
	/**
	 * Last received command velocity from teleoperation
	 */
	geometry_msgs::Twist _teleoperation_cmd_vel;

	void publishCmdVel();
	void publishOperationMode();
	bool setOperationMode(statemachine_msgs::SetOperationMode::Request &req,
			statemachine_msgs::SetOperationMode::Response &res);
	void autonomyCmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	void teleoperationCmdVelCallback(
			const geometry_msgs::Twist::ConstPtr& cmd_vel);
	void teleoperationIdleTimerCallback(const ros::TimerEvent& event);

};

} /* namespace statemachine */

#endif /* ROBOTCONTROLMUX_H_ */
