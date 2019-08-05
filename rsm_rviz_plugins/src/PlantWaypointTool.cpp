#include <rsm_rviz_plugins/PlantWaypointTool.h>

namespace rsm {

PlantWaypointTool::PlantWaypointTool() :
		moving_flag_node_( NULL), current_flag_property_( NULL) {
	shortcut_key_ = 'w';
	ros::NodeHandle nh("rsm");
	_add_waypoint_client = nh.serviceClient<rsm_msgs::AddWaypoint>(
			"addWaypoint");
}

PlantWaypointTool::~PlantWaypointTool() {
	for (unsigned i = 0; i < flag_nodes_.size(); i++) {
		scene_manager_->destroySceneNode(flag_nodes_[i]);
	}
}

void PlantWaypointTool::onInitialize() {
	flag_resource_ = "package://rsm_rviz_plugins/media/flag.dae";

	if (rviz::loadMeshFromResource(flag_resource_).isNull()) {
		ROS_ERROR("PlantWaypointTool: failed to load model resource '%s'.",
				flag_resource_.c_str());
		return;
	}

	moving_flag_node_ =
			scene_manager_->getRootSceneNode()->createChildSceneNode();
	Ogre::Entity* entity = scene_manager_->createEntity(flag_resource_);
	moving_flag_node_->attachObject(entity);
	moving_flag_node_->setVisible(false);
}

void PlantWaypointTool::activate() {
	if (moving_flag_node_) {
		moving_flag_node_->setOrientation(Ogre::Quaternion());
		moving_flag_node_->setVisible(true);

		current_flag_property_ = new rviz::VectorProperty(
				"Waypoint " + QString::number(flag_nodes_.size()));
		current_flag_property_->setReadOnly(true);
		getPropertyContainer()->addChild(current_flag_property_);
		state_ = Moving;
	}
}

void PlantWaypointTool::deactivate() {
	if (moving_flag_node_) {
		moving_flag_node_->setVisible(false);
		delete current_flag_property_;
		current_flag_property_ = NULL;
	}
}

int PlantWaypointTool::processMouseEvent(rviz::ViewportMouseEvent& event) {
	if (!moving_flag_node_) {
		return Render;
	}
	Ogre::Vector3 intersection;
	Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
	if (rviz::getPointOnPlaneFromWindowXY(event.viewport, ground_plane, event.x,
			event.y, intersection)) {
		moving_flag_node_->setVisible(true);
		if (state_ == Moving) {
			moving_flag_node_->setPosition(intersection);
			current_flag_property_->setVector(intersection);
		}
		if (event.leftDown()) {
			state_ == Position;
			pos_ = intersection;
			moving_flag_node_->setPosition(pos_);
			state_ = Orientation;
			return Render;
		} else if (event.type == QEvent::MouseMove && event.left()
				&& state_ == Orientation) {
			//compute angle in x-y plane
			double angle = atan2(intersection.y - pos_.y,
					intersection.x - pos_.x);
			moving_flag_node_->setVisible(true);

			//we need base_orient, since the arrow goes along the -z axis by default (for historical reasons)
			moving_flag_node_->setOrientation(
					Ogre::Quaternion(Ogre::Radian(angle),
							Ogre::Vector3::UNIT_Z));
			return Render;
		} else if (event.leftUp() && state_ == Orientation) {
			//compute angle in x-y plane
			double angle = atan2(intersection.y - pos_.y,
					intersection.x - pos_.x);
			makeFlag(pos_, angle);
			return (Finished | Render);
		}
	} else {
		moving_flag_node_->setVisible(false); // If the mouse is not pointing at the ground plane, don't show the flag.
		return Render;
	}
}

void PlantWaypointTool::makeFlag(const Ogre::Vector3& position, double angle) {
	rsm_msgs::AddWaypoint srv;
	rsm_msgs::Waypoint waypoint;
	waypoint.pose.position.x = position.x;
	waypoint.pose.position.y = position.y;
	waypoint.pose.position.z = position.z;
	waypoint.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
	srv.request.waypoint = waypoint;
	srv.request.position = -1;
	if (!_add_waypoint_client.call(srv)) {
		ROS_ERROR("Failed to call Add Waypoint service");
	}
}


void PlantWaypointTool::save(rviz::Config config) const {
}

void PlantWaypointTool::load(const rviz::Config& config) {
}

} // end namespace rsm

PLUGINLIB_EXPORT_CLASS(rsm::PlantWaypointTool, rviz::Tool)
