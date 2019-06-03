#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> MoveBaseActionServer;

MoveBaseActionServer* as(NULL);

void navigation_goal_callback(const move_base_msgs::MoveBaseGoalConstPtr& frontier_goal)
{
    as->setSucceeded(move_base_msgs::MoveBaseResult(), "Goal reached.");
    //ROS_INFO("Frontier goal intercepted: %3.3f %3.3f", frontier_goal->target_pose.pose.position.x, frontier_goal->target_pose.pose.position.y);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "goalInterceptNode");
    ros::NodeHandle nh;
    as = new MoveBaseActionServer(nh, "frontier_move_base", navigation_goal_callback, false);
    as->start();
    while(ros::ok())
    {
        ros::spin();
    }
    return 0;
}
