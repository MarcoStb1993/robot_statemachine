#include "RSMControls.h"

namespace rsm {

RSMControlPanel::RSMControlPanel(QWidget* parent) :
		rviz::Panel(parent), _gui(new Ui::rsm_controls()) {
	_gui->setupUi(this);
	initCommunications();
	connectSlots();
	_exploration_running = false;
	_waypoint_following_running = false;
	_emergency_stop_active = false;
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
	initRoutineComboBox();
	getStateInfo();
}

void RSMControlPanel::initCommunications() {
	ros::NodeHandle nh("rsm");
	_start_stop_exploration_client = nh.serviceClient<std_srvs::SetBool>(
			"startStopExploration");
	_set_exploration_mode_client = nh.serviceClient<std_srvs::SetBool>(
			"setExplorationMode");

	_start_stop_waypoint_following_client = nh.serviceClient<std_srvs::SetBool>(
			"startStopWaypointFollowing");
	_waypoint_reset_client = nh.serviceClient<std_srvs::Trigger>(
			"resetWaypoints");
	_set_waypoint_following_mode_client = nh.serviceClient<
			rsm_msgs::SetWaypointFollowingMode>(
			"setWaypointFollowingMode");

	_state_info_subscriber = nh.subscribe("stateInfo", 10,
			&RSMControlPanel::stateInfoCallback, this);
	_state_info_client = nh.serviceClient<std_srvs::Trigger>("stateInfo");
	_set_operation_mode_client = nh.serviceClient<
			rsm_msgs::SetOperationMode>("setOperationMode");
	_operation_mode_subcriber = nh.subscribe<rsm_msgs::OperationMode>(
			"operationMode", 1,
			&RSMControlPanel::operationModeCallback, this);

	_set_reverse_mode_client = nh.serviceClient<std_srvs::SetBool>(
			"setReverseMode");
	_reverse_mode_subscriber = nh.subscribe<std_msgs::Bool>("reverseMode", 10,
			&RSMControlPanel::reverseModeCallback, this);
	_stop_2d_nav_goal_client = nh.serviceClient<std_srvs::Trigger>(
			"stop2DNavGoal");

	_add_waypoint_client = nh.serviceClient<rsm_msgs::AddWaypoint>(
			"addWaypoint");
	_get_waypoint_routines_client = nh.serviceClient<
			rsm_msgs::GetWaypointRoutines>("getWaypointRoutines");

	_get_robot_pose_client = nh.serviceClient<rsm_msgs::GetRobotPose>(
			"getRobotPose");
}

void RSMControlPanel::connectSlots() {
connect(_gui->start_exploration_button, SIGNAL(clicked(bool)), this, SLOT(startStopExploration()));

connect(_gui->start_waypoint_following_button, SIGNAL(clicked(bool)), this, SLOT(startStopWaypointFollowing()));
connect(_gui->reset_waypoints_button, SIGNAL(clicked(bool)), this, SLOT(resetWaypoints()));

connect(_gui->set_waypoint_button, SIGNAL(clicked(bool)), this, SLOT(addWaypoint()));

connect(_gui->reverse_checkbox, SIGNAL(clicked(bool)), this, SLOT(setReverseMode()));

connect(_gui->emergency_stop_button, SIGNAL(clicked(bool)), this, SLOT(emergencyStop()));
connect(_gui->stopped_radio_button, SIGNAL(clicked(bool)), this, SLOT(stopOperation()));
connect(_gui->autonomy_radio_button, SIGNAL(clicked(bool)), this, SLOT(setAutonomyOperation()));
connect(_gui->teleoperation_radio_button, SIGNAL(clicked(bool)), this, SLOT(setTeleoperation()));

connect(_gui->stop_2d_nav_goal_button, SIGNAL(clicked(bool)), this, SLOT(stop2dNavGoal()));
}

void RSMControlPanel::startStopExploration() {
std_srvs::SetBool srv;
if (_exploration_running) {
	srv.request.data = false;
} else {
	srv.request.data = true;
}
if (_start_stop_exploration_client.call(srv)) {
	//ROS_INFO("State was changed: %s -> message: %s",	srv.response.success ? "true" : "false", srv.response.message.c_str()	);
	if (srv.response.success) {
		if (_exploration_running) {
			_gui->start_exploration_button->setText("Start");
			_exploration_running = false;
			_gui->exploration_info_text->setText("Exploration stopped");
			_gui->exploration_mode_box->setEnabled(true);
		} else {
			_gui->start_exploration_button->setText("Stop");
			_exploration_running = true;
			_gui->exploration_info_text->setText("Exploration running");
			setExplorationMode();
			_gui->exploration_mode_box->setEnabled(false);
		}
	} else {
		QString text = QString("Call unsuccessful: %1").arg(
				srv.response.message.c_str());
		_gui->exploration_info_text->setText(text);
	}
} else {
	ROS_ERROR("Failed to call service Start/Stop Exploration");
	_gui->exploration_info_text->setText("Exploration service not available");
}
}

void RSMControlPanel::startStopWaypointFollowing() {
std_srvs::SetBool srv;
if (_waypoint_following_running) {
	srv.request.data = false;
} else {
	srv.request.data = true;
}
if (_start_stop_waypoint_following_client.call(srv)) {
	//ROS_INFO("State was changed: %s -> message: %s",	srv.response.success ? "true" : "false", srv.response.message.c_str()	);
	if (srv.response.success) {
		if (_waypoint_following_running) {
			_gui->start_waypoint_following_button->setText("Start");
			_waypoint_following_running = false;
			_gui->waypoint_following_info_text->setText(
					"Waypoint following stopped");
			_gui->reset_waypoints_button->setEnabled(true);
			_gui->mode_box->setEnabled(true);
		} else {
			_gui->start_waypoint_following_button->setText("Stop");
			_waypoint_following_running = true;
			_gui->waypoint_following_info_text->setText("Waypoint running");
			_gui->reset_waypoints_button->setEnabled(false);
			_gui->mode_box->setEnabled(false);
			setWaypointFollowingMode();
		}
	} else {
		QString text = QString("Call unsuccessful: %1").arg(
				srv.response.message.c_str());
		_gui->waypoint_following_info_text->setText(text);
	}
} else {
	ROS_ERROR("Failed to call service Start/Stop Waypoint Following");
	_gui->waypoint_following_info_text->setText(
			"Waypoint Following service not available");
}
}

void RSMControlPanel::resetWaypoints() {
std_srvs::Trigger srv;
if (_waypoint_reset_client.call(srv)) {
	//ROS_INFO("State was changed: %s -> message: %s",	srv.response.success ? "true" : "false", srv.response.message.c_str()	);
	if (srv.response.success) {
		_gui->waypoint_following_info_text->setText("Waypoints reset");
	} else {
		QString text = QString("Reset unsuccessful: %1").arg(
				srv.response.message.c_str());
		_gui->waypoint_following_info_text->setText(text);
	}
} else {
	ROS_ERROR("Failed to call service Reset Waypoints");
	_gui->waypoint_following_info_text->setText(
			"Reset Waypoints service not available");
}
}

void RSMControlPanel::addWaypoint() {
rsm_msgs::GetRobotPose srv;
if (!_get_robot_pose_client.call(srv)) {
	ROS_ERROR("Failed to call Get Robot Pose service");
	_gui->waypoint_following_info_text->setText(
			"Get Robot Pose service not available");
} else {
	rsm_msgs::AddWaypoint srv2;
	rsm_msgs::Waypoint waypoint;
	waypoint.pose = srv.response.pose;
	if (_gui->routine_combo_box->currentIndex() == 0) {
		waypoint.routine = "";
	} else {
		waypoint.routine =
				_gui->routine_combo_box->currentText().toUtf8().constData();
	}
	srv2.request.waypoint = waypoint;
	srv2.request.position = -1;
	if (!_add_waypoint_client.call(srv2)) {
		ROS_ERROR("Failed to call Add Waypoint service");
		_gui->waypoint_following_info_text->setText(
				"Add Waypoint service not available");
	}
}
}

void RSMControlPanel::setReverseMode() {
std_srvs::SetBool srv;
srv.request.data = !_reverse_mode;
if (_set_reverse_mode_client.call(srv)) {
	if (srv.response.success) {
		_reverse_mode = !_reverse_mode;
		_gui->reverse_checkbox->setChecked(_reverse_mode);
	} else {
		_gui->movement_label->setText("Control: Failed to change reverse mode");
	}
} else {
	ROS_ERROR("Failed to call Set Reverse Mode service");
	_gui->movement_label->setText(
			"Control: Set Reverse Mode service not available");
}
}

void RSMControlPanel::emergencyStop() {
_emergency_stop_active = !_emergency_stop_active;
_operation_mode = rsm_msgs::OperationMode::STOPPED;
callSetOperationMode();
}

void RSMControlPanel::stopOperation() {
if (_operation_mode != rsm_msgs::OperationMode::STOPPED) {
	_operation_mode = rsm_msgs::OperationMode::STOPPED;
	callSetOperationMode();
}
}

void RSMControlPanel::setAutonomyOperation() {
if (_operation_mode != rsm_msgs::OperationMode::AUTONOMOUS) {
	_operation_mode = rsm_msgs::OperationMode::AUTONOMOUS;
	callSetOperationMode();
}
}

void RSMControlPanel::setTeleoperation() {
if (_operation_mode != rsm_msgs::OperationMode::TELEOPERATION) {
	_operation_mode = rsm_msgs::OperationMode::TELEOPERATION;
	callSetOperationMode();
}
}

void RSMControlPanel::stop2dNavGoal() {
std_srvs::Trigger srv;
if (!_stop_2d_nav_goal_client.call(srv)) {
	ROS_ERROR("Failed to call Stop 2D Nav Goal service");
	_gui->movement_label->setText(
			"Control: Stop 2D Nav Goal service not available");
}
}

void RSMControlPanel::callSetOperationMode() {
rsm_msgs::SetOperationMode srv;
srv.request.operationMode.emergencyStop = _emergency_stop_active;
srv.request.operationMode.mode = _operation_mode;
if (_set_operation_mode_client.call(srv)) {
	updateOperationModeGUI();
} else {
	ROS_ERROR("Failed to call service Set Operation Mode");
	_gui->control_label->setText(
			"Control: Set Operation Mode service not available");
}
}

void RSMControlPanel::setWaypointFollowingMode() {
ROS_INFO("set waypoint following mode");
rsm_msgs::SetWaypointFollowingMode srv;
srv.request.mode = _gui->mode_box->currentIndex();
if (_set_waypoint_following_mode_client.call(srv)) {
	if (!srv.response.success) {
		QString text =
				QString("Set Waypoint Following Mode unsuccessful: %1").arg(
						srv.response.message.c_str());
		_gui->waypoint_following_info_text->setText(text);
	}

} else {
	ROS_ERROR("Failed to call service Set Waypoint Following Mode");
	_gui->waypoint_following_info_text->setText(
			"Set Waypoint Following Mode service not available");
}
}

void RSMControlPanel::setExplorationMode() {
std_srvs::SetBool srv;
srv.request.data = _gui->exploration_mode_box->currentIndex();
if (!_set_exploration_mode_client.call(srv)) {
	ROS_ERROR("Failed to call service Set Operation Mode");
	_gui->exploration_info_text->setText(
			"Set Exploration Mode service not available");
}
}

void RSMControlPanel::stateInfoCallback(
	const std_msgs::String::ConstPtr& state_info) {
QString text = QString("Current state: %1").arg(state_info->data.c_str());
_gui->current_state_text->setText(text);
if (state_info->data.compare("Idle") == 0
		|| state_info->data.compare("Teleoperation") == 0
		|| state_info->data.compare("Emergency Stop") == 0
		|| state_info->data.compare("Navigation: Simple Goal") == 0) {
	_gui->start_exploration_button->setText("Start");
	_exploration_running = false;
	_gui->exploration_info_text->setText("");
	_gui->exploration_mode_box->setEnabled(true);
	_gui->start_waypoint_following_button->setText("Start");
	_gui->reset_waypoints_button->setEnabled(true);
	_gui->mode_box->setEnabled(true);
	_waypoint_following_running = false;
	_gui->waypoint_following_info_text->setText("");
} else if (state_info->data.compare("Calculate Goal") == 0
		|| state_info->data.compare("Navigation: Exploration") == 0
		|| state_info->data.compare("Mapping") == 0) {
	if (!_exploration_running) {
		_gui->start_exploration_button->setText("Stop");
		_exploration_running = true;
		_gui->exploration_info_text->setText("Exploration running");
		_gui->exploration_mode_box->setEnabled(false);
	}
} else if (state_info->data.compare("Waypoint Following") == 0
		|| state_info->data.compare("Navigation: Waypoint Following") == 0) {
	if (!_waypoint_following_running) {
		_gui->start_waypoint_following_button->setText("Stop");
		_waypoint_following_running = false;
		_gui->waypoint_following_info_text->setText(
				"Waypoint Following running");
	}
}
}

void RSMControlPanel::reverseModeCallback(
	const std_msgs::Bool::ConstPtr& reverse_mode) {
_reverse_mode = reverse_mode->data;
_gui->reverse_checkbox->setChecked(_reverse_mode);
}

void RSMControlPanel::operationModeCallback(
	const rsm_msgs::OperationMode::ConstPtr& operation_mode) {
_emergency_stop_active = operation_mode->emergencyStop;
_operation_mode = operation_mode->mode;
updateOperationModeGUI();
}

void RSMControlPanel::initRoutineComboBox() {
QStringList list;
list.append("None");
rsm_msgs::GetWaypointRoutines srv;
if (_get_waypoint_routines_client.call(srv)) {
	_waypoint_routines = srv.response.waypointRoutines;
	for (auto it : _waypoint_routines) {
		list.append(it.c_str());
	}
} else {
	ROS_ERROR("Failed to call Get Waypoint Routines service");
}
_gui->routine_combo_box->addItems(list);
}

void RSMControlPanel::getStateInfo() {
std_srvs::Trigger srv;
if (_state_info_client.call(srv)) {
	QString text = QString("Current state: %1").arg(
			srv.response.message.c_str());
	_gui->current_state_text->setText(text);
} else {
	ROS_ERROR("Failed to call State Info service");
}
}

void RSMControlPanel::updateOperationModeGUI() {
_gui->stopped_radio_button->setChecked(_operation_mode == 0);
_gui->autonomy_radio_button->setChecked(_operation_mode == 1);
_gui->teleoperation_radio_button->setChecked(_operation_mode == 2);
if (_emergency_stop_active) {
	_gui->emergency_stop_button->setStyleSheet(
			"background-color: rgba(150,75,75,1);color: rgba(0,0,0,1);");
	_gui->autonomy_radio_button->setEnabled(false);
	_gui->teleoperation_radio_button->setEnabled(false);
} else {
	_gui->emergency_stop_button->setStyleSheet(
			"background-color: rgba(150,0,0,1);color: rgba(255,255,255,1);");
	_gui->autonomy_radio_button->setEnabled(true);
	_gui->teleoperation_radio_button->setEnabled(true);
}
}

void RSMControlPanel::save(rviz::Config config) const {
rviz::Panel::save(config);
}

void RSMControlPanel::load(const rviz::Config& config) {
rviz::Panel::load(config);
}

}  // end namespace moveit_dashboard

PLUGINLIB_EXPORT_CLASS(rsm::RSMControlPanel, rviz::Panel)
