#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <rsm_msgs/WaypointArray.h>
#include <rsm_msgs/Waypoint.h>
#include <rsm_msgs/MoveWaypoint.h>
#include <rsm_msgs/RemoveWaypoint.h>
#include <rsm_msgs/SetWaypointRoutine.h>
#include <rsm_msgs/GetWaypointRoutines.h>
#include <tf/tf.h>

namespace rsm {

/**
 * @class WaypointFollowingVisualization
 * @brief Class for visualizing waypoints as interactive markers in RViz
 * 		  and handling their interaction
 */
class WaypointFollowingVisualization {
public:

	/**
	 * Constructor
	 */
	WaypointFollowingVisualization();
	/**
	 * Desctructor
	 */
	~WaypointFollowingVisualization();

private:
	/**
	 * Method being called when moving the marker and changing it's position
	 * in the array calling ServiceProvider's service
	 * @param feedback
	 */
	void processFeedback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	/**
	 * Method being called when selecting delete from the marker's menu,
	 * deletes the waypoint from the array calling ServiceProvider's service
	 * @param feedback Feedback object generated from click
	 */
	void removeWaypoint(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	/**
	 * Method being called when selecting a routine from the marker's menu,
	 * changes the routine for the respective waypoint from the array calling
	 * ServiceProvider's service
	 * @param feedback Feedback object generated from click
	 */
	void setWaypointRoutine(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	/**
	 * Callback for receiving the waypoint array published by the ServiceProvider
	 * @param waypoint_array
	 */
	void waypointCallback(
			const rsm_msgs::WaypointArray::ConstPtr& waypoint_array);
	/**
	 * Draws the interactive marker for the waypoints with position waypoint_pos in the waypoint array
	 * @param waypoint_pos Position in the waypoint array of the waypoint to visualize
	 */
	void addWaypointMarkerToServer(int waypoint_pos);
	/**
	 * Timer callback for one-shot timer to initialize the menu handler with a delay,
	 * so the ServiceProvider's services will be available
	 * @param event Timer event
	 */
	void timerCallback(const ros::TimerEvent& event);
	/**
	 * Compare if waypoints send from the ServiceProvider match the ones here
	 * regarding array length, routine, visited- and unreachable-status.
	 * @param waypoint_array received from ServiceProvider
	 * @return True if the waypoint array has changed, false otherwise
	 */
	bool waypointArrayChanged(
			const rsm_msgs::WaypointArray::ConstPtr& waypoint_array);
	/**
	 * Timer callback for periodically refreshing the displayed markers
	 * @param event Timer event
	 */
	void periodicalRefreshTimerCallback(const ros::TimerEvent& event);

	interactive_markers::InteractiveMarkerServer _waypoint_server;
	interactive_markers::MenuHandler _menu_handler;
	interactive_markers::MenuHandler::EntryHandle _routine_switch_handler;

	ros::Subscriber _waypoint_subscriber;
	ros::ServiceClient _move_waypoint_client;
	ros::ServiceClient _remove_waypoint_client;
	ros::ServiceClient _set_waypoint_routine_client;
	ros::ServiceClient _get_waypoint_routines_client;

	/**
	 * Waypoint array
	 */
	rsm_msgs::WaypointArray _waypoints;
	/**
	 * List of all available waypoint routines
	 */
	std::vector<std::string> _waypoint_routines;
	/**
	 * If the waypoint markers should be redrawn on the next callback
	 */
	bool _refresh_waypoint_markers;
	/**
	 * Timer to call menu handle initialization with a delay so ServiceProvider's services will be available
	 */
	ros::Timer _service_call_delay_timer;
	/**
	 * Timer to periodically refresh the drawn markers to synchronize with the waypoint array in case of high system load
	 */
	ros::Timer _periodical_refresh_timer;
};

}
