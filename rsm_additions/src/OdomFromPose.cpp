#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>

bool publish_transform; //should a transform between odom and base_footprint be published

ros::Time stamp;                      //last time stamp of the current pose of the robot
std::string frame_id, child_frame_id; //frames for the transform
geometry_msgs::Pose pose;             //last pose of the robot
bool pose_received;                   //new pose was received from the scan matcher

double prev_x, prev_y, prev_th; //previous position and orientation of the robot
ros::Time prev_stamp;           //time stamp of last pose

double cmd_vel[3]; //command velocities used for calculation method of odom velocity

/* Receive and save current velocity command */
void get_cmd_vel(const geometry_msgs::Twist::ConstPtr &vel_msg)
{
    cmd_vel[0] = vel_msg->linear.x;
    cmd_vel[1] = vel_msg->linear.y;
    cmd_vel[2] = vel_msg->angular.z;
}

/* Receive laser scan matchers pose stamped */
void get_pose2D(const geometry_msgs::PoseStamped::ConstPtr &poseStamped)
{
    stamp = poseStamped->header.stamp;
    pose = poseStamped->pose;
    pose_received = true;
}

/* Calculate and publish odometry from difference between current and last pose of the robot */
void publish_odom(ros::Publisher odom_pub, tf::TransformBroadcaster odom_broadcaster)
{
    double dt = (prev_stamp - stamp).toSec();
    double delta_x = prev_x - pose.position.x;
    double delta_y = prev_y - pose.position.y;
    //calculate theta from quaternion
    tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, th;
    m.getRPY(roll, pitch, th);

    double delta_th = prev_th - th;

    double vx, vy, vth;
    //calculate velocities from position difference
    if (cmd_vel[2] != 0)
    {
        // calculate vx from circular path, vy=0
        if (cmd_vel[0] != 0 && delta_th != 0)
        {
            vx = (sqrt(delta_x * delta_x + delta_y * delta_y) * delta_th) / (2 * sin(delta_th / 2) * dt);
            if (cmd_vel[0] < 0)
            {
                vx *= -1;
            }
        }
        else
        {
            vx = (cos(th) * delta_x - sin(th) * delta_y) / dt;
        }
        vy = 0;
        vth = delta_th / dt;
    }
    else
    {
        // calculate vx and vy from position difference (expect vth to be 0)
        double cos_th = cos(th);
        double sin_th = sin(th);
        vx = (cos_th * delta_x - sin_th * delta_y) / dt;
        vy = (sin_th * delta_x + cos_th * delta_y) / dt;
        vth = delta_th / dt;
    }

    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = pose.orientation.x;
    odom_quat.y = pose.orientation.y;
    odom_quat.z = pose.orientation.z;
    odom_quat.w = pose.orientation.w;

    if (publish_transform)
    {
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = stamp;
        odom_trans.header.frame_id = frame_id;
        odom_trans.child_frame_id = child_frame_id;
        odom_trans.transform.translation.x = pose.position.x;
        odom_trans.transform.translation.y = pose.position.y;
        odom_trans.transform.translation.z = 0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);
    }

    odom.header.stamp = stamp;
    odom.header.frame_id = frame_id;
    odom.child_frame_id = child_frame_id;
    odom.pose.pose.position.x = pose.position.x;
    odom.pose.pose.position.y = pose.position.y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = odom_quat;

    odom.pose.covariance[0] = 1e-6;
    odom.pose.covariance[7] = 1e-6;
    odom.pose.covariance[14] = 1e6;
    odom.pose.covariance[21] = 1e6;
    odom.pose.covariance[28] = 1e6;
    odom.pose.covariance[35] = 1e-6;

    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    odom.twist.covariance[0] = 1e-3;
    odom.twist.covariance[7] = 1e-3;
    odom.twist.covariance[14] = 1e6;
    odom.twist.covariance[21] = 1e6;
    odom.twist.covariance[28] = 1e6;
    odom.twist.covariance[35] = 1e-3;

    odom_pub.publish(odom);
    prev_stamp = stamp;
    prev_x = pose.position.x;
    prev_y = pose.position.y;
    prev_th = th;
    pose_received = false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odomFromPose");

    ros::NodeHandle n;

    ros::NodeHandle nh("~");
    publish_transform = false;
    nh.getParam("publish_transform", publish_transform);

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("scan/odom", 20);
    ros::Subscriber pose_sub;
    ros::Subscriber vel_sub = n.subscribe("cmd_vel", 20, get_cmd_vel);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Rate r(20);
    pose_sub = n.subscribe("pose_stamped", 20, get_pose2D);
    frame_id = "odom";
    child_frame_id = "base_footprint";
    pose_received = false;

    cmd_vel[0] = 0.0;
    cmd_vel[1] = 0.0;
    cmd_vel[2] = 0.0;

    prev_x = 0.0;
    prev_y = 0.0;
    prev_th = 0.0;
    prev_stamp = ros::Time::now();

    while (ros::ok())
    {

        ros::spinOnce();
        if (pose_received)
        {
            publish_odom(odom_pub, odom_broadcaster);
        }
        r.sleep();
    }
    return 0;
}