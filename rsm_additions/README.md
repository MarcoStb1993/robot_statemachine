# RSM Additions

Additions to the RSM including all mandatory plugin states and a plugin routine 
state. Also an additional service provider is included.

## Documentation

This package implements plugins for the [Calculate Goal State](#calculate-goal-state), 
the [Navigation State](#navigation-state) and the [Mapping State](#mapping-state). Furthermore, 
the [Reversing Routine State](#reversing-routine-state) is an optional routine state 
plugin. The [Additions Service Provider](#additions-service-provider) handles the 
data to be transferred between these plugins.

### Calculate Goal State

The Calculate Goal State interfaces the ROS package [explore lite](http://wiki.ros.org/explore_lite),
subscribes to it's visualization topic that shows frontiers on a 2D map and extracts the closest frontier center 
point to the robot as navigation goal. Therefore, it retrieves the robot's pose and 
calculates it's distance to each of the frontier's center points. Also, previously 
failed goals are disregarded as potential navigation goals. If it fails to find a suitable goal 
for exploration, it returns an error message and transitions back to the [Idle State](../rsm_core#non-customizable-states)

To have [explore lite](http://wiki.ros.org/explore_lite) running without directly sending 
commands to the [navigation stack](http://wiki.ros.org/navigation), a mock action server 
is constructed in the [Additions Service Provider](#additions-service-provider) that leads 
the exploration to believe the goals are accepted. Otherwise it does not start to to 
calculate frontiers. Furthermore, [explore lite](http://wiki.ros.org/explore_lite) 
is launched with `progress_timeout` set to 3600 seconds which gives the robot ten hours 
to move, otherwise the exploration stops and needs to be relaunched.

### Navigation State

The Navigation State realizes an interface to the [navigation ROS package](http://wiki.ros.org/navigation).
It forwards received goals to the navigation stack and also gets feedback from it regarding the progress. 
If it fails, the goal is added to the failed goals list. If it succeeds, the failed 
goal list will be reset.

When standing still for too long, it transitions to the [Idle State](../rsm_core#non-customizable-states).
Reaching the goal will initiate a transition to the [Mapping State](#mapping-state) 
or the particular routine state if there is one available. If not, [Waypoint Following State](../rsm_core#non-customizable-states)
is called. After reaching a navigation goal provided by RViz and if waypoint following has ended,
it transitions to [Idle State](../rsm_core#non-customizable-states).

Reverse driving is realised by running two navigation stacks, one for forward driving 
and one for reverse driving. This is explained in more detail [later](#reverse-robot-movement-with-navigation-stack).
For reverse driving the robot also features a transform 
to a reverse base frame. When driving in reverse, all output command velocities are 
negated by the [Additions Service Provider](#additions-service-provider). If the reverse mode
is activated or deactivated, the goal is cancelled and sent to the reverse navigation.

### Mapping State

There are two mapping plugins included. The first state is just a dummy state while the latter is swiveling a simulated Intel RealSense camera from left to right around a revoluting joint.

#### Mapping Dummy State

The Mapping Dummy State is just transitioning back to the Calculate Goal State as
specific mapping procedures are only relevant for the particular robot.

#### RealSense Mapping State

Swivels an Intel RealSense camera mounted on a joint revoluting around the z-axis from left to right and back to it's centered position to map the surrounding area. This only works for the implemented Gazebo simulation as it publishes commands to the joint the RealSense is mounted on.

### Reversing Routine State

A Routine State called Reversing Routine is also include and toggles the reverse mode when
the routine is executed. This means the robot is driving in reverse when it was going forward
before and vice versa.

### Additions Service Provider

This data handler class retrieved the frontiers published by [explore lite](http://wiki.ros.org/explore_lite) 
for visualization, extracts each frontier's center and republishes them as possible 
exploration goals. In case, the exploration mode is set to *Interrupt*, it is also
checked if the current navigation goal is still in the list of exploration goals.
If not, it is published that the goal is obsolete. A tolerance for comparing these
positions can be set with a parameter.

Furthermore, goals that could not be reached during exploration, here named as
failed goals, are saved and published. These serve as a way of blacklisting goals.
This normally happens, when the navigation finishes.

For driving in reverse mode, the velocity commands issued by the reverse navigation 
stack are also subscribed to and republished with negated linear velocities. It also 
provides a service that is called when reverse mode should be activated. Since nothing 
needs to be changed in the configuration to change to reverse mode, this service just 
replies that it was successful.

If the Realsense mapping is interrupted, a service is provided that moves the camera back to it's centered
position while the RSM is continuing.

## Examples

An example to move a robot backwards and forwards with the [navigation stack](http://wiki.ros.org/navigation).

### Reverse robot movement with navigation stack

The following code needs to be included in your launch file (or the nodes launched respectively) 
to allow the robot to navigate in forward and reverse depending on the set mode:

```xml
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        ...
        <param name="global_costmap/robot_base_frame" value="$(arg robot_frame)" />
        <param name="local_costmap/robot_base_frame" value="$(arg robot_frame)" />
        <remap from="/cmd_vel" to="$(arg autonomy_cmd_vel_topic)" />
    </node>

    <node pkg="move_base" type="move_base" respawn="false" name="move_base_reverse" output="screen">
        ...
        <param name="global_costmap/robot_base_frame" value="$(arg robot_frame)_reverse" />
        <param name="local_costmap/robot_base_frame" value="$(arg robot_frame)_reverse" />
        <remap from="/cmd_vel" to="$(arg autonomy_cmd_vel_topic)_reverse" />
        <remap from="move_base/goal" to="move_base_reverse/goal" />
        <remap from="move_base/cancel" to="move_base_reverse/cancel" />
        <remap from="move_base/feedback" to="move_base_reverse/feedback" />
        <remap from="move_base/status" to="move_base_reverse/status" />
        <remap from="move_base/result" to="move_base_reverse/result" />
    </node>

    <node pkg="tf" type="static_transform_publisher" name="base_footprint_reverse" args="0 0 0 3.1415 0 0 (arg robot_frame) $(arg robot_frame)_reverse 10" />
```

Replace the dots with the usual parameters for the navigation stack. Both packages share the same parameters but for the robot base frame. To use a particular 
navigation stack, initiate it like in this [tutorial](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) 
and call the action for forward movement with "move_base" and for reverse movement 
with "move\_base\_reverse".

## Nodes

### additionalServiceProviderNode

An additional data handler class that adds services to interface the exploration lite and navigation packages.

#### Published Topics

**<_autonomy_cmd_vel_topic>** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))  
Topic name for the autonomy command velocity

**explorationGoals** ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))  
List of all currently available exploration goals

**failedGoals** ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))  
List of all previously failed goals

**realsense_controller/command** ([std_msgs/Float64](http://docs.ros.org/api/std_msgs/html/msg/Float64.html))  
Position the Realsense revolute joint will move to

**goalObsolete** ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))  
Information if the current goal is still viable (only active is exploration mode is set to *Interrupt*)

#### Subscribed Topics

**<autonomy_cmd_vel_top>_reverse** ([std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html))  
Topic name for the autonomy command velocity in reverse mode

**explore/frontiers** ([visualization_msgs/MarkerArray](http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html))  
All frontier grid cells as points and closest frontier points as spheres

**explorationMode** ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html))  
The current exploration mode (true: interrupt, false: finish)

**explorationGoalStatus** ([rsm_msgs/GoalStatus](../rsm_msgs/msg/GoalStatus.msg))  
The currently active goal's status and pose

#### Services

**setNavigationToReverse** ([std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html))  
Needs to be implemented for reverse mode, just returns success

#### Parameters

**~update_frequency** (float, default: 20)  
Update rate in Hz

**~autonomy_cmd_vel_topic** (string, default: "autonomy/cmd_vel")  
Topic name for the autonomy command velocity

**~calculate_goal_plugin** (string, default: "rsm::CalculateGoalPlugin")  
Sets the plugin's name for the calculate goal state.

**~navigation_plugin** (string, default: "rsm::NavigationPlugin")  
Sets the plugin's name for the navigation state.

**~mapping_plugin** (string, default: "rsm::MappingDummyPlugin")  
Sets the plugin's name for the mapping state.

**~exploration_goal_tolerance** (double, default: 0.05)  
Distance in all directions in meters that the robot's current position can differ from an exploration goal to still count it as reached
