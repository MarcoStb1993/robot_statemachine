#include <rsm_additions/TiltScannerMappingState.h>

namespace rsm
{

	TiltScannerMappingState::TiltScannerMappingState()
	{
	}

	TiltScannerMappingState::~TiltScannerMappingState()
	{
	}

	void TiltScannerMappingState::onSetup()
	{
		_tilt_scan_subscriber = _nh.subscribe("cloud2", 10,
											  &TiltScannerMappingState::tiltScanCallback, this);
		_start_tilt_scan_client = _nh.serviceClient<ohm_sensors_msgs::StartTiltScan3D>(
			"scanparamsSrv");
		_sensor_head_pan_pos_subscriber = _nh.subscribe("pan/pos/present", 10, &TiltScannerMappingState::sensorHeadPanPosCallback, this);
		_sensor_head_tilt_pos_subscriber = _nh.subscribe("tilt/pos/present", 10, &TiltScannerMappingState::sensorHeadTiltPosCallback, this);

		_sensor_head_pan_vel_publisher = _nh.advertise<std_msgs::Float32>("pan/speed/des", 10, true);
		_sensor_head_pan_pos_publisher = _nh.advertise<std_msgs::Float32>("pan/pos/des", 10, true);
		_sensor_head_tilt_pos_publisher = _nh.advertise<std_msgs::Float32>("til/pos/des", 10, true);

		_sensor_head_state = at_start;

		ros::NodeHandle nh("rsm");
		_navigation_goal_completed_service = nh.serviceClient<
			rsm_msgs::GoalCompleted>("navigationGoalCompleted");
		_name = "E: Tilt Scanner 3D Mapping";
		_tilt_scan_complete = false;
		_navigation_completed_status = rsm_msgs::GoalStatus::ABORTED;
	}

	void TiltScannerMappingState::onEntry()
	{
		publishSensorHeadPanVel(PAN_VELOCITY);
		_sensor_head_state = moving_left;

		ohm_sensors_msgs::StartTiltScan3D srv;
		srv.request.speed = 30;
		srv.request.startPosition = 90;
		srv.request.endPosition = 180;
		if (!_start_tilt_scan_client.call(srv))
		{
			ROS_ERROR("Failed to call Start Tilt Scan");
			_navigation_completed_status = rsm_msgs::GoalStatus::FAILED;
			_stateinterface->transitionToVolatileState(_stateinterface->getPluginState(CALCULATEGOAL_STATE));
		}
		else
		{
			_tilt_scan_timeout = _nh.createTimer(ros::Duration(10.0), &TiltScannerMappingState::timerCallback, this);
		}
	}

	void TiltScannerMappingState::onActive()
	{
		if (_sensor_head_state == at_left)
		{
			publishSensorHeadPanVel(-PAN_VELOCITY);
			_sensor_head_state = moving_right;
		}
		else if (_sensor_head_state == at_right)
		{
			publishSensorHeadPanPos(0.0);
			_sensor_head_state = return_to_start;
		}
		else if (_sensor_head_state == return_to_start && _tilt_scan_complete)
		{

			_navigation_completed_status = rsm_msgs::GoalStatus::REACHED;
			_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(CALCULATEGOAL_STATE));
		}
	}

	void TiltScannerMappingState::onExit()
	{
		if (_sensor_head_state != return_to_start)
		{
			publishSensorHeadPanPos(0.0);
			_sensor_head_state = return_to_start;
		}
		rsm_msgs::GoalCompleted srv;
		srv.request.status.goal_status = _navigation_completed_status;
		if (!_navigation_goal_completed_service.call(srv))
		{
			ROS_ERROR("Failed to call Complete Navigation Goal service");
		}
	}

	void TiltScannerMappingState::onExplorationStart(bool &success,
													 std::string &message)
	{
		success = false;
		message = "Exploration running";
	}

	void TiltScannerMappingState::onExplorationStop(bool &success,
													std::string &message)
	{
		success = true;
		message = "Exploration stopped";
		_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
	}

	void TiltScannerMappingState::onWaypointFollowingStart(bool &success,
														   std::string &message)
	{
		success = false;
		message = "Exploration running";
	}

	void TiltScannerMappingState::onWaypointFollowingStop(bool &success,
														  std::string &message)
	{
		success = false;
		message = "Exploration running";
	}

	void TiltScannerMappingState::onInterrupt(int interrupt)
	{
		switch (interrupt)
		{
		case EMERGENCY_STOP_INTERRUPT:
			_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
			_interrupt_occured = true;
			break;
		case TELEOPERATION_INTERRUPT:
			_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
			_interrupt_occured = true;
			break;
		case SIMPLE_GOAL_INTERRUPT:
			_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(NAVIGATION_STATE));
			_interrupt_occured = true;
			break;
		}
	}

	void TiltScannerMappingState::publishSensorHeadPanVel(float vel)
	{
		std_msgs::Float32 pan_speed;
		pan_speed.data = vel;
		_sensor_head_pan_vel_publisher.publish(pan_speed);
	}

	void TiltScannerMappingState::publishSensorHeadPanPos(float pos)
	{
		std_msgs::Float32 pan_pos;
		pan_pos.data = pos;
		_sensor_head_pan_pos_publisher.publish(pan_pos);
	}

	void TiltScannerMappingState::publishSensorHeadTiltPos(float pos)
	{
		std_msgs::Float32 tilt_pos;
		tilt_pos.data = pos;
		_sensor_head_tilt_pos_publisher.publish(tilt_pos);
	}

	void TiltScannerMappingState::tiltScanCallback(sensor_msgs::PointCloud2::ConstPtr cloud)
	{
		_tilt_scan_complete = true;
	}

	void TiltScannerMappingState::sensorHeadPanPosCallback(std_msgs::Float32::ConstPtr pos)
	{
		if (_sensor_head_state == moving_left && pos->data >= 1.57)
		{
			publishSensorHeadPanVel(0.0);
			_sensor_head_state = at_left;
		}
		else if (_sensor_head_state == moving_right && pos->data <= -1.57)
		{
			publishSensorHeadPanVel(0.0);
			_sensor_head_state = at_right;
		}
	}

	void TiltScannerMappingState::sensorHeadTiltPosCallback(std_msgs::Float32::ConstPtr pos)
	{
		ROS_ERROR_STREAM("sensor head tilt pos current: " << pos->data);
		if (abs(pos->data) >= 0.01) //~0.57 degrees variation from horizontal
			publishSensorHeadTiltPos(0.0);
	}

	void TiltScannerMappingState::timerCallback(const ros::TimerEvent &event)
	{
		_navigation_completed_status = rsm_msgs::GoalStatus::FAILED;
		_stateinterface->transitionToVolatileState(
			_stateinterface->getPluginState(CALCULATEGOAL_STATE));
	}

}

PLUGINLIB_EXPORT_CLASS(rsm::TiltScannerMappingState, rsm::BaseState)
