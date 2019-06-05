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

#include <statemachine_msgs/AddWaypoint.h>
#include <statemachine_msgs/GetWaypoints.h>
#include <statemachine_msgs/RemoveWaypoint.h>
#include <statemachine_msgs/WaypointVisited.h>

namespace Ogre {
class SceneNode;
class Vector3;
}

namespace rviz {
class VectorProperty;
class VisualizationManager;
class ViewportMouseEvent;
}

namespace statemachine {

class PlantWaypointTool: public rviz::Tool {

public:
	PlantWaypointTool();
	~PlantWaypointTool();

	void onInitialize();

	void activate();
	void deactivate();

	int processMouseEvent(rviz::ViewportMouseEvent& event);

	void load(const rviz::Config& config);
	void save(rviz::Config config) const;

private:
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
