#ifndef MAPPINGSTATE_H
#define MAPPINGSTATE_H

#include <pluginlib/class_list_macros.h>
#include <rsm_core/BaseState.h>
#include <rsm_core/IdleState.h>
#include <rsm_core/EmergencyStopState.h>
#include <rsm_core/TeleoperationState.h>
#include <rsm_core/StateInterface.h>
#include <rsm_msgs/GoalCompleted.h>
#include <ohm_tilt_scanner_3d/SrvScanParams.h>
#include <sensor_msgs/PointCloud2.h>

namespace rsm {

/**
 * @class   TiltScannerMappingState
 * @brief   Starts a tilt scan to obtain a 3D point cloud.
 */
class TiltScannerMappingState: public BaseState {

public:

	/**
	 * Constructor
	 */
	TiltScannerMappingState();

	/**
	 * Destructor
	 */
	~TiltScannerMappingState();

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
	ros::Subscriber _tilt_scan_subscriber;
	ros::ServiceClient _start_tilt_scan_client;
	ros::ServiceClient _navigation_goal_completed_service;
	ros::Timer _tilt_scan_timeout;

	/**
	 * Did the tilt scan finish
	 */
	bool _tilt_scan_complete;
	/**
	 * Was the mapping at the exploration goal successful or not
	 */
	int _navigation_completed_status;

	/**
	 * Callback for point cloud from tilt scan that show that it finished
	 * @param cloud Point cloud message
	 */
	void tiltScanCallback(sensor_msgs::PointCloud2::ConstPtr cloud);

	/**
	 * Callback for the timer to detect a tilt scan timeout 
	 */
	void timerCallback(const ros::TimerEvent& event);
};

}

#endif
