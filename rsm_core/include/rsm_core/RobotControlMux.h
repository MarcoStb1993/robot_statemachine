#ifndef ROBOTCONTROLMUX_H_
#define ROBOTCONTROLMUX_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <rsm_msgs/OperationMode.h>
#include <rsm_msgs/SetOperationMode.h>

/**
 * Threshold for minimal movement of joysticks to be recognized as and intended command
 */
#define MOVE_THRESH 0.05

namespace rsm {

/**
 * @class RobotControlMux
 * @brief Handles all command velocities and only allows the robot to move
 * 		  at all if no emergency stop is set and only move autonomous if autonomous operation is
 * 		  active
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
	/**
	 * Publishes msgs from all publishers in this class (cmd vel and operation mode)
	 */
	void publishTopics();

private:
	ros::NodeHandle _nh;
	ros::ServiceServer _set_operation_mode_service;
	ros::Subscriber _teleoperation_cmd_vel_sub;
	ros::Subscriber _autonomy_cmd_vel_sub;
	ros::Subscriber _joystick_sub;
	ros::Publisher _cmd_vel_pub;
	ros::Publisher _operation_mode_pub;
	ros::Timer _teleoperation_idle_timer;

	std::string _teleoperation_cmd_vel_topic;
	std::string _autonomy_operation_cmd_vel_topic;
	std::string _cmd_vel_topic;
	std::string _joystick_topic;

	/**
	 * Time until teleoperation mode will be stopped when no new message is received
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
	 * Is a joystick in use for controlling the robot
	 */
	bool _joystick_used;
	/**
	 * Last received command velocity from autonomy
	 */
	geometry_msgs::Twist _autonomy_cmd_vel;
	/**
	 * Last received command velocity from teleoperation
	 */
	geometry_msgs::Twist _teleoperation_cmd_vel;
	/**
	 * Last received joystick command
	 */
	sensor_msgs::Joy _joystick_cmd;

	/**
	 * Publish the cmd vel controlling the robot depending on the current operation mode
	 */
	void publishCmdVel();
	/**
	 * Publish current operation mode
	 */
	void publishOperationMode();
	/**
	 * Callback for receiving the cmd vel produced by autonomous operation
	 * @param cmd_vel Cmd vel from autonomous operation
	 */
	void autonomyCmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel);
	/**
	 * Callback for receiving the cmd vel from teleoperation and checks if a command not equals zero was issued
	 * @param cmd_vel Cmd vel from teleoperation
	 */
	void teleoperationCmdVelCallback(
			const geometry_msgs::Twist::ConstPtr& cmd_vel);
	/**
	 * Callback for receiving the commands issued from a connected joystick and checks if a new command was sent
	 * @param joy Joystick commands
	 */
	void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy);
	/**
	 * Timer callback for checking if teleoperation was stopped and robot is idle again
	 * @param event
	 */
	void teleoperationIdleTimerCallback(const ros::TimerEvent& event);
	/**
	 * Checks if the given joystick command will make the robot move and is different to the previous one
	 * @param joy Joystick commands
	 * @return If the new command will lead to a robot action
	 */
	bool checkJoystickCommand(const sensor_msgs::Joy::ConstPtr& joy);
	bool setOperationMode(rsm_msgs::SetOperationMode::Request &req,
			rsm_msgs::SetOperationMode::Response &res);

};

} /* namespace rsm */

#endif /* ROBOTCONTROLMUX_H_ */
