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
		ros::NodeHandle nh("rsm");
		_start_tilt_scan_client = nh.serviceClient<ohm_tilt_scanner_3d::SrvScanParams>(
			"scanparamsSrv");
		_navigation_goal_completed_service = nh.serviceClient<
			rsm_msgs::GoalCompleted>("navigationGoalCompleted");
		_name = "E: Tilt Scanner 3D Mapping";
		_tilt_scan_complete = false;
		_navigation_completed_status = rsm_msgs::GoalStatus::ABORTED;
	}

	void TiltScannerMappingState::onEntry()
	{
		ohm_tilt_scanner_3d::SrvScanParams srv;
		srv.request.speed = 40;
		srv.request.startPosition = 90;
		srv.request.endPosition = 170;
		if (!_start_tilt_scan_client.call(srv))
		{
			ROS_ERROR("Failed to call Start Tilt Scan");
			_navigation_completed_status = rsm_msgs::GoalStatus::FAILED;
			_stateinterface->transitionToVolatileState(_stateinterface->getPluginState(CALCULATEGOAL_STATE));
		}
	}

	void TiltScannerMappingState::onActive()
	{
		
	}

	void TiltScannerMappingState::onExit()
	{
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

	void TiltScannerMappingState::tiltScanCallback(sensor_msgs::PointCloud2::ConstPtr cloud)
	{
		_navigation_completed_status = rsm_msgs::GoalStatus::REACHED;
		_stateinterface->transitionToVolatileState(
			_stateinterface->getPluginState(CALCULATEGOAL_STATE));
	}

}

PLUGINLIB_EXPORT_CLASS(rsm::TiltScannerMappingState, rsm::BaseState)
