#ifndef RSM_CONTROLS_H
#define RSM_CONTROLS_H

#include <ros/ros.h>
#include <rviz/panel.h>
#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>
#include <QStringList>
#include <QDebug>
#include <QObject>
#include "ui_rsm_controls.h"
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <rsm_msgs/OperationMode.h>
#include <rsm_msgs/SetWaypointFollowingMode.h>
#include <rsm_msgs/AddWaypoint.h>
#include <rsm_msgs/GetWaypointRoutines.h>
#include <rsm_msgs/GetRobotPose.h>
#include <rsm_msgs/SetOperationMode.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <pluginlib/class_list_macros.h>

namespace rsm {

/**
 * @class   RSMControlPanel
 * @brief   Panel plugin for RViz which adds buttons to interface the RSM
 */
class RSMControlPanel: public rviz::Panel {
	Q_OBJECT
public:
	/**
	 * @brief Constructor
	 * @param Parent widget
	 */
	RSMControlPanel(QWidget *parent = 0);

	/**
	 * @brief Saves configuration data from this panel to the Config object
	 * @param Config object to save configuration to
	 */
	virtual void load(const rviz::Config &config);
	/**
	 * @brief Load configuration data for this panel from Config object
	 * @param Config object to load configuration from
	 */
	virtual void save(rviz::Config config) const;

protected slots:
	void startStopExploration();
	void startStopWaypointFollowing();
	void resetWaypoints();
	void addWaypoint();
	void setReverseMode();
	void emergencyStop();
	void stopOperation();
	void setAutonomyOperation();
	void setTeleoperation();
	void stop2dNavGoal();

private:
	Ui::rsm_controls* _gui;

	ros::NodeHandle _nh;

	ros::ServiceClient _start_stop_exploration_client;
	ros::ServiceClient _start_stop_waypoint_following_client;
	ros::ServiceClient _waypoint_reset_client;
	ros::ServiceClient _set_waypoint_following_mode_client;
	ros::ServiceClient _add_waypoint_client;
	ros::ServiceClient _get_waypoint_routines_client;
	ros::ServiceClient _set_reverse_mode_client;
	ros::ServiceClient _get_robot_pose_client;
	ros::ServiceClient _set_operation_mode_client;
	ros::ServiceClient _set_exploration_mode_client;
	ros::ServiceClient _state_info_client;
	ros::ServiceClient _stop_2d_nav_goal_client;
	ros::Subscriber _state_info_subscriber;
	ros::Subscriber _reverse_mode_subscriber;
	ros::Subscriber _operation_mode_subcriber;
	ros::Subscriber _exploration_mode_subscriber;

	/**
	 * @brief Was an operation mode button just pushed
	 */
	bool _operation_mode_button_pushed;
	/**
	 * @brief Is the exploration currently running
	 */
	bool _exploration_running;
	/**
	 * @brief Is waypoint following currently running
	 */
	bool _waypoint_following_running;
	/**
	 * Is currently driving in reverse mode
	 */
	bool _reverse_mode;
	/**
	 * Is emergency stop currently activated
	 */
	bool _emergency_stop_active;
	/**
	 * Currently active mode of operation (0=stopped, 1=autonomous, 2=teleoperation)
	 */
	int _operation_mode;
	/**
	 * List of all available waypoint routines
	 */
	std::vector<std::string> _waypoint_routines;

	void callSetOperationMode();
	void stateInfoCallback(const std_msgs::String::ConstPtr& state_info);
	void reverseModeCallback(const std_msgs::Bool::ConstPtr& reverse_mode);
	void operationModeCallback(
			const rsm_msgs::OperationMode::ConstPtr& operation_mode);
	void initCommunications();
	void connectSlots();
	void initRoutineComboBox();
	void getStateInfo();
	void setWaypointFollowingMode();
	void setExplorationMode();
	void updateOperationModeGUI();
};

}  // end namespace rsm

#endif  // RSM_CONTROLS_H
