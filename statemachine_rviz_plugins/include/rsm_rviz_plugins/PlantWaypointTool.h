#ifndef PLANT_WAYPOINT_TOOL_H_
#define PLANT_WAYPOINT_TOOL_H_

#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/visualization_manager.h>
#include <rviz/mesh_loader.h>
#include <rviz/geometry.h>
#include <rviz/properties/vector_property.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <tf/transform_datatypes.h>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <rsm_msgs/AddWaypoint.h>
#include <rsm_msgs/GetWaypoints.h>
#include <rsm_msgs/RemoveWaypoint.h>
#include <rsm_msgs/WaypointVisited.h>

namespace Ogre {
class SceneNode;
class Vector3;
}

namespace rviz {
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace rsm {

/**
 * @class PlantWaypointTool
 * @brief Serves as a Tool plugin for RViz which enables planting waypoints
 * 		  on a desired location with an adjustable orientation.
 */
class PlantWaypointTool: public rviz::Tool {

public:
	/**
	 * Constructor
	 */
	PlantWaypointTool();
	/**
	 * Destructor
	 */
	~PlantWaypointTool();
	/**
	 * Initializes plugin
	 */
	void onInitialize();
	/**
	 * Called when tool is activated
	 */
	void activate();
	/**
	 * Called when tool is deactivated
	 */
	void deactivate();

	/**
	 * Processing mouse events in RViz while tool is active
	 * @param event Mouse event in RViz
	 * @return Render or Finished
	 */
	int processMouseEvent(rviz::ViewportMouseEvent& event);

	/**
	 * Load config (not used)
	 * @param config
	 */
	void load(const rviz::Config& config);
	/**
	 * Save config (not used)
	 * @param config
	 */
	void save(rviz::Config config) const;

private:
	/**
	 * Calls AddWaypoint service with the desired position and orientation for the new waypoint
	 * @param position Position of the new waypoint
	 * @param angle Yaw angle of the new waypoint
	 */
	void makeFlag(const Ogre::Vector3& position, double angle);

	std::vector<Ogre::SceneNode*> flag_nodes_;
	Ogre::SceneNode* moving_flag_node_;
	std::string flag_resource_;
	rviz::VectorProperty* current_flag_property_;

	ros::ServiceClient _add_waypoint_client;

	enum State {
		Moving, Position, Orientation
	};
	State state_;
	Ogre::Vector3 pos_;
};

}

#endif /* PLANT_WAYPOINT_TOOL_H_ */
