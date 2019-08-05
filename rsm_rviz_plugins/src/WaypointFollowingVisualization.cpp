#include <rsm_rviz_plugins/WaypointFollowingVisualization.h>

namespace rsm {

WaypointFollowingVisualization::WaypointFollowingVisualization() :
		_waypoint_server("waypoint_marker") {
	ros::NodeHandle nh("rsm");
	_waypoint_subscriber = nh.subscribe<rsm_msgs::WaypointArray>(
			"waypoints", 10, &WaypointFollowingVisualization::waypointCallback,
			this);
	_move_waypoint_client = nh.serviceClient<rsm_msgs::MoveWaypoint>(
			"moveWaypoint");
	_remove_waypoint_client =
			nh.serviceClient<rsm_msgs::RemoveWaypoint>(
					"removeWaypoint");
	_set_waypoint_routine_client = nh.serviceClient<
			rsm_msgs::SetWaypointRoutine>("setWaypointRoutine");
	_get_waypoint_routines_client = nh.serviceClient<
			rsm_msgs::GetWaypointRoutines>("getWaypointRoutines");
	_menu_handler.insert("Delete",
			boost::bind(&WaypointFollowingVisualization::removeWaypoint, this,
					_1));
	_service_call_delay_timer = nh.createTimer(ros::Duration(2.0),
			&WaypointFollowingVisualization::timerCallback, this, true);
	_periodical_refresh_timer = nh.createTimer(ros::Duration(1.0),
			&WaypointFollowingVisualization::periodicalRefreshTimerCallback,
			this, false);
	_refresh_waypoint_markers = false;
}

WaypointFollowingVisualization::~WaypointFollowingVisualization() {
}

void WaypointFollowingVisualization::processFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	rsm_msgs::MoveWaypoint srv;
	srv.request.position = std::stoi(feedback->marker_name);
	srv.request.pose = feedback->pose;
	if (_move_waypoint_client.call(srv)) {
		//ROS_INFO("Successful call to service Move Waypoint");
	} else {
		ROS_ERROR("Failed to call service Move Waypoint");
	}
}

void WaypointFollowingVisualization::removeWaypoint(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	rsm_msgs::RemoveWaypoint srv;
	srv.request.position = std::stoi(feedback->marker_name);
	if (_remove_waypoint_client.call(srv)) {
		//ROS_INFO("Successful call to service Remove Waypoint");
	} else {
		ROS_ERROR("Failed to call service Remove Waypoint");
	}
}

void WaypointFollowingVisualization::setWaypointRoutine(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
	rsm_msgs::SetWaypointRoutine srv;
	srv.request.position = std::stoi(feedback->marker_name);
	if (feedback->menu_entry_id == 3) {	//Set to none
		srv.request.routine = "";
	} else {
		srv.request.routine = _waypoint_routines[feedback->menu_entry_id - 4]; //three previous menu entries
	}
	if (!_set_waypoint_routine_client.call(srv)) {
		ROS_ERROR("Failed to call service Move Waypoint");
	}

}

void WaypointFollowingVisualization::waypointCallback(
		const rsm_msgs::WaypointArray::ConstPtr& waypoint_array) {
	if (_refresh_waypoint_markers || waypointArrayChanged(waypoint_array)) {
		_waypoints = *waypoint_array;
		_waypoint_server.clear();
		for (int i = 0; i < _waypoints.waypoints_size; i++) {
			addWaypointMarkerToServer(i);
		}
		_waypoint_server.applyChanges();
		_refresh_waypoint_markers = false;
	}
}

void WaypointFollowingVisualization::addWaypointMarkerToServer(
		int waypoint_pos) {
	visualization_msgs::InteractiveMarker waypoint_marker;
	waypoint_marker.header.frame_id = "map";
	waypoint_marker.name = std::to_string(waypoint_pos);
	if (!_waypoints.waypoints[waypoint_pos].routine.empty()) {
		std::ostringstream s;
		s << waypoint_marker.name << ": "
				<< _waypoints.waypoints[waypoint_pos].routine;
		waypoint_marker.description = s.str();
	} else {
		waypoint_marker.description = waypoint_marker.name;
	}
	waypoint_marker.pose = _waypoints.waypoints[waypoint_pos].pose;

// create a colorized flag pole
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
	box_marker.mesh_resource =
			"package://rsm_rviz_plugins/media/flag.dae";
	box_marker.mesh_use_embedded_materials = false;
	box_marker.scale.x = 1.0;
	box_marker.scale.y = 1.0;
	box_marker.scale.z = 1.0;
	if (_waypoints.waypoints[waypoint_pos].unreachable) {
		box_marker.color.r = 1.0;
	} else if (_waypoints.waypoints[waypoint_pos].visited) {
		box_marker.color.g = 1.0;
	} else {
		box_marker.color.b = 1.0;
	}
	box_marker.color.a = 1.0;

// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.markers.push_back(box_marker);

// add the control to the interactive marker
	waypoint_marker.controls.push_back(box_control);

	visualization_msgs::InteractiveMarkerControl menu_control;
	menu_control.name = "menu_control";
	menu_control.interaction_mode =
			visualization_msgs::InteractiveMarkerControl::MENU;
	menu_control.always_visible = true;
	menu_control.markers.push_back(box_marker);
	waypoint_marker.controls.push_back(menu_control);

// create a control which will move the box
// this control does not contain any markers,
// which will cause RViz to insert two arrows
	visualization_msgs::InteractiveMarkerControl move_control;
	move_control.name = "move_control";
	tf::Quaternion orientation(0.0, 1.0, 0.0, 1.0);
	orientation.normalize();
	tf::quaternionTFToMsg(orientation, move_control.orientation);
	move_control.interaction_mode =
			visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE;
	waypoint_marker.controls.push_back(move_control);
	move_control.interaction_mode =
			visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	waypoint_marker.controls.push_back(move_control);

// add the interactive marker to our collection &
// tell the _waypointServer to call processFeedback() when feedback arrives for it
	_waypoint_server.insert(waypoint_marker);
	_waypoint_server.setCallback(waypoint_marker.name,
			boost::bind(&WaypointFollowingVisualization::processFeedback, this,
					_1));
	_menu_handler.apply(_waypoint_server, waypoint_marker.name);
}

void WaypointFollowingVisualization::timerCallback(
		const ros::TimerEvent& event) {
	interactive_markers::MenuHandler::EntryHandle sub_menu_handle =
			_menu_handler.insert("Set routine");
	_menu_handler.insert(sub_menu_handle, "None",
			boost::bind(&WaypointFollowingVisualization::setWaypointRoutine,
					this, _1));
	rsm_msgs::GetWaypointRoutines srv;
	if (_get_waypoint_routines_client.call(srv)) {
		_waypoint_routines = srv.response.waypointRoutines;
		for (auto it : _waypoint_routines) {
			_menu_handler.insert(sub_menu_handle, it,
					boost::bind(
							&WaypointFollowingVisualization::setWaypointRoutine,
							this, _1));
		}
	} else {
		ROS_ERROR("Failed to call Get Waypoint Routines service");
	}
}

bool WaypointFollowingVisualization::waypointArrayChanged(
		const rsm_msgs::WaypointArray::ConstPtr& waypoint_array) {
	if (waypoint_array->waypoints_size == _waypoints.waypoints_size) {
		for (int i = 0; i < _waypoints.waypoints_size; i++) {
			if (_waypoints.waypoints[i].routine.compare(
					waypoint_array->waypoints[i].routine) != 0
					|| _waypoints.waypoints[i].visited
							!= waypoint_array->waypoints[i].visited
					|| _waypoints.waypoints[i].unreachable
							!= waypoint_array->waypoints[i].unreachable) {
				return true;
			}
		}
		return false;
	} else {
		return true;
	}
}

void WaypointFollowingVisualization::periodicalRefreshTimerCallback(
		const ros::TimerEvent& event) {
	_refresh_waypoint_markers = true;
}

}
