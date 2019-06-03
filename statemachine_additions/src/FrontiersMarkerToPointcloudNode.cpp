#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>

ros::Subscriber frontiers_marker_array_subscriber;
ros::Publisher frontiers_pointcloud_publisher;
int seq;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void frontierCallback(
		const visualization_msgs::MarkerArray::ConstPtr& frontiers) {
	PointCloud pointcloud;
	for (auto it : frontiers->markers) {
		if (it.type == visualization_msgs::Marker::POINTS) {
			int frontier_size = it.points.size();
			if (frontier_size > 0) {
				pcl::PointXYZ point;
				point.x = it.points[frontier_size / 2].x;
				point.y = it.points[frontier_size / 2].y;
				point.z = it.points[frontier_size / 2].z;
				pointcloud.points.push_back(point);
			}
		}
	}
	sensor_msgs::PointCloud2 frontier_cloud;
	pcl::toROSMsg(pointcloud, frontier_cloud);
	frontier_cloud.header.seq = seq++;
	frontier_cloud.header.frame_id = "map";
	frontier_cloud.header.stamp = ros::Time::now();
	frontiers_pointcloud_publisher.publish(frontier_cloud);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "frontiersMarkerToPointcloudNode");
	ros::NodeHandle nh;
	frontiers_marker_array_subscriber = nh.subscribe("explore/frontiers", 10,
			frontierCallback);
	frontiers_pointcloud_publisher = nh.advertise<PointCloud>(
			"/explore_server/explore_costmap/explore_boundary/frontiers", 10);
	seq = 0;
	while (ros::ok()) {
		ros::spin();
	}
	return 0;
}
