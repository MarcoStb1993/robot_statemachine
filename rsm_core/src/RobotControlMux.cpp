#include <rsm_core/RobotControlMux.h>

namespace rsm {

RobotControlMux::RobotControlMux() {
	ros::NodeHandle privateNh("~");
	privateNh.param<std::string>("autonomy_cmd_vel_topic",
			_autonomy_operation_cmd_vel_topic, "autonomy/cmd_vel");
	privateNh.param<std::string>("teleoperation_cmd_vel_topic",
			_teleoperation_cmd_vel_topic, "teleoperation/cmd_vel");
	privateNh.param<std::string>("cmd_vel_topic", _cmd_vel_topic, "cmd_vel");
	privateNh.param("teleoperation_idle_timer",
			_teleoperation_idle_timer_duration, 0.5);

	ros::NodeHandle nh("rsm");
	_set_operation_mode_service = nh.advertiseService("setOperationMode",
			&RobotControlMux::setOperationMode, this);

	_autonomy_cmd_vel_sub = _nh.subscribe(_autonomy_operation_cmd_vel_topic, 1,
			&RobotControlMux::autonomyCmdVelCallback, this);
	_teleoperation_cmd_vel_sub = _nh.subscribe(_teleoperation_cmd_vel_topic, 1,
			&RobotControlMux::teleoperationCmdVelCallback, this);

	_cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(_cmd_vel_topic, 1);
	_operation_mode_pub = nh.advertise<rsm_msgs::OperationMode>(
			"operationMode", 1);

	_teleoperation_idle_timer = _nh.createTimer(
			ros::Duration(_teleoperation_idle_timer_duration),
			&RobotControlMux::teleoperationIdleTimerCallback, this, false,
			false);

	_emergency_stop_active = 0;
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
}

RobotControlMux::~RobotControlMux() {

}

void RobotControlMux::publishTopics() {
	publishCmdVel();
	publishOperationMode();
}

void RobotControlMux::publishCmdVel() {
	geometry_msgs::Twist cmd_vel;
	if (!_emergency_stop_active) {
		if (_operation_mode == rsm_msgs::OperationMode::AUTONOMOUS) {
			cmd_vel = _autonomy_cmd_vel;
		} else if (_operation_mode
				== rsm_msgs::OperationMode::TELEOPERATION) {
			cmd_vel = _teleoperation_cmd_vel;
		}
	}
	_cmd_vel_pub.publish(cmd_vel);
}

void RobotControlMux::publishOperationMode() {
	rsm_msgs::OperationMode msg;
	msg.emergencyStop = _emergency_stop_active;
	msg.mode = _operation_mode;
	_operation_mode_pub.publish(msg);
}

bool RobotControlMux::setOperationMode(
		rsm_msgs::SetOperationMode::Request &req,
		rsm_msgs::SetOperationMode::Response &res) {
	_emergency_stop_active = req.operationMode.emergencyStop;
	_operation_mode = req.operationMode.mode;
	res.success = 1;
	res.message = "Operation mode set";
	return true;
}

void RobotControlMux::autonomyCmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	_autonomy_cmd_vel = *cmd_vel;
}

void RobotControlMux::teleoperationCmdVelCallback(
		const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	_teleoperation_cmd_vel = *cmd_vel;
	if (!_emergency_stop_active) {
		if (_teleoperation_cmd_vel.linear.x != 0.0
				|| _teleoperation_cmd_vel.linear.y != 0.0
				|| _teleoperation_cmd_vel.linear.z != 0.0
				|| _teleoperation_cmd_vel.angular.x != 0.0
				|| _teleoperation_cmd_vel.angular.y != 0.0
				|| _teleoperation_cmd_vel.angular.z != 0.0) {
			_operation_mode = rsm_msgs::OperationMode::TELEOPERATION;
			_teleoperation_idle_timer.stop();
		}
		if (_operation_mode == rsm_msgs::OperationMode::TELEOPERATION) {
			_teleoperation_idle_timer.start();
		}
	}
}

void RobotControlMux::teleoperationIdleTimerCallback(
		const ros::TimerEvent& event) {
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
	geometry_msgs::Twist empty_cmd_vel;
	_teleoperation_cmd_vel = empty_cmd_vel;
	_teleoperation_idle_timer.stop();
}

} /* namespace rsm */
