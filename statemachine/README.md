#  Statemachine

The statemachine's core components will be explained first and it's usage afterwards, including examples and tutorials for writing plugins, including them into the statemachine and setting up a robot.

## Documentation

The statemachine consists of various non-customizable and custom states that are based on the [Base State](#base-state). The former [non-customizable states](#non-customizable-states) and the [Base State](#base-state) are a part of this package containing the statemachine's basics.  
To handle state transitions the [State Interface](#state-interface) is used. [Robot Control Mux](#robot-control-mux) coordinates the actual control of the robot's movement while the [Service Provider](#service-provider) contains services, publishers and subscribers for communication between states and updating the GUI. To be able to handle arbitrary robots, the statemachine relies on [Plugins](#plugins) that can be implemented depending on the robot.

### Base State

The base state for all states of the statemachine features the following four main functions:
* onSetup
* onEntry
* onActive
* onExit

The function `onSetup` is called immediately after it was constructed and should be used to initialize the state. The function `onEntry` is run before the state's `onActive` method is executed for the first time and should be used to start up the processing in the state. The latter is the state's primary method that is executed periodically and contains it's main logic. `onExit` is called before the state will be destroyed and should take care of leaving the state cleanly.

To realize interrupts in the statemachine, the following five functions need to be implemented:
* onExplorationStart
* onExplorationStop
* onWaypointFollowingStart
* onWaypointFollowingStop
* onInterrupt

These functions handle commands issued from the GUI or the use of teleoperation by telling the current state which command or interrupt occurred and let the state handle it. The method `onInterrupt` receives the type of interrupt which are also defined in the Base State and are listed below:
* INTERRUPT\_END: Former interrupt ended (only relevant to specific interrupt handlers)
* EMERGENCY\_STOP\_INTERRUPT: Emergency Software Stop was pushed in the GUI
* TELEOPERATION\_INTERRUPT: Teleoperation was used
* SIMPLE\_GOAL\_INTERRUPT: A navigation goal was issued through the RViz GUI

The four other methods receive a reference to a bool and a string variable. The former informs if the request was successful and the desired action will be executed (true) or not (false) and the latter features a descriptive text.

The Base State holds a reference to the [State Interface](#state-interface) which has to be used for state transitions. It also has a variable with it's name that is necessary to display the current state in the GUI and needs to be set in the `onSetup` or `onEntry` method.

### State Interface

The State Interface holds a reference to the current state and handles state transitions. It also provides references to plugins created for exploration, navigation, mapping or routines to the states.
The State Interface provides the method `transitionToVolatileState` which will initiate a transition to the state provided as an argument. The provided argument is a `boost::shared_ptr` of the Base State type. This can be one of the known [non-customizable states](#non-customizable-states) or a custom state defined through a plugin. 

To access these plugins State Interface offers the method `getPluginState` which takes the plugin type and optionally a plugin routine name as parameters. The former can be one of the following types:
* CALCULATEGOAL\_STATE
* NAVIGATION\_STATE
* MAPPING\_STATE
* ROUTINE\_STATE

For a *ROUTINE_STATE* the routine name needs to be provided as well, otherwise this parameter can remain empty. The other plugin states are set by parameters provided to State Interface on launch. If no plugin type was specified but only a name, arbitrary plugins can be created and returned for state transition. If no plugin type and name were received or the desired plugin to be created does not exist, the **Idle State** will be returned and an error message put out. 

State Interface subscribes to the *stateInfo* and *simpleGoal* topics to issue interrupts to the currently active state. Furthermore, it offers the two services *startStopExploration* and *startStopWaypointFollowing* which call the particular function in the active state.

The State Interface updates the currently active state periodically though it's `awake` function. This function also executes the state transition initiated by `transitionToVolatileState` and calls the active state's methods.

### Robot Control Mux

The Robot Control Mux (=Multiplexer) controls the velocity commands sent to the ROS node interfacing the motor controllers. In a simple configuration a navigation or teleoperation node would output velocity commands that will be received by the motor controller interface and move the robot. To enable high level control of the input the motor controller receives, the Robot Control Mux should be the only node in the setup publishing directly on the topic the motor controller interface subscribes to.  

Velocity commands generated by navigation should be published to an autonomy topic and velocity commands issued by teleoperation to a teleoperation topic. These two topics are subscribed by the Robot Control Mux that decides which or if any topic will be conducted. The two input and the output topic's name are set by parameters at launch.

Which topic will be conducted is based on the operation mode which can be one of the following:
* Autonomy
* Stopped
* Teleoperation

For *Autonomy* and *Teleoperation* the respective topic is propagated to the motor controller interface. If the operation mode is set to *Stopped* a command velocity of zero for all directions is published. The operation mode can be set through the GUI by a service Robot Control Mux is providing. It is published to the GUI for display as well. If a teleoperation command is issued, the mode automatically switches to *Teleoperation*. When in *Teleoperation* mode, a timer is started to supervise if new commands are being issued. If no new commands are received for the timer duration (which is set through a parameter), *Teleoperation* is replaced with the *Stopped* mode.

If the software emergency stop is activated in the GUI, the operation mode is handled as *Stopped* and cannot be changed until the stop button is released again.

### Service Provider

The Service Provider handles the communication between the different states and saves data throughout state transitions. Therefore it offers a lot of services to save and retrieve variables for the core functionality of the statemachine.

It offers all services to control waypoint following which include adding, moving and removing single waypoints, setting their `visited` and `unreachable` variables and the routine to be executed upon reaching the waypoint. Furthermore, all waypoints can be retrieved and reset which effectively sets `visited` and `unreachable` to false. The waypoint following mode can be set and the list of all available routines retrieved. The latter is given as a parameter to the Service Provider. The list of waypoints is also published.

For setting and retrieving the current navigation goal the Service Provider is offering services. In addition previously failed goals can be set, retrieved or reset. These serve as a way of blacklisting goals.

The current robot pose can be retrieved and is calculated from the transform from the map to the robot's base footprint.

The Service Provider also hosts services for exploration that enable setting and getting the exploration mode. It is also published. If the exploration mode is set to *interrupt*, the Service Provider subscribes to the list of available frontiers and checks if the current navigation goal is still in this list. A tolerance for comparing these positions can be set with a parameter. If the navigation goal is not a frontier anymore, it becomes obsolete. This info is published when the mode is set to *interrupt* as well.

Furthermore, it advertises services for setting and retrieving the reverse mode, which is also published. 

### Non-customizable states

The core statemachine already features the following states for direct usage:
* **Boot State:** Is the first state to be called and subscribes to a service which tells it when all necessary systems are available and ready to use. Then it initiates a transition to the **Idle State**. Can only be interrupted by the software emergency stop. 
* **Emergency Stop State:** State being called when the software emergency stop was pushed. Only allows transition to **Idle State** when button is released.
* **Idle State:** Standard state when no commands were issued. Allows transitions to all other states through interrupts.
* **Teleoperation State:** State being called when teleoperation commands were issued. Only transitions to **Idle State** when teleoepration timed out and **Emergency Stop State** when receiving the particular interrupt.
* **Waypoint Following State:** Handles the waypoint following functionality by providing the next navigation goal depending on the status of all waypoints and the waypoint following mode. Normally transitions to the navigation state plugin. Can be interrupted by the software emergency stop and teleoperation which leads to a transition to the particular state. If waypoint following is stopped, transitions to **Idle State**.

### Plugins

The statemachine package requires three different plugin states, one for exploration to calculate the next goal, one for navigation and one for mapping. The first is called when exploration is started or a previous exploration target was mapped successfully and  should interface an exploration package like [explore lite](http://wiki.ros.org/explore_lite) which finds unexplored regions in the map and extract a next goal from it. The second should interface a package for navigation like the [ROS navigation stack](http://wiki.ros.org/navigation) and update the statemachine according to the navigation's progress. The last is called when an exploration goal is reached and can include movements for better map acquisition or similar behaviors.

Also, up to ten plugins states can be included for the waypoint following routines that are executed upon reaching a waypoint. They are not necessary for the statemachine like the plugins mentioned above. These routine can be implemented to enable arbitrary behavior when reaching a certain waypoint, for example inspecting gauge valves with a camera.

More plugins can be added if additional states during exploration or waypoint following are desired. These can only be called from other implemented plugin states as the basic statemachine online includes transitions to the plugins described above. For example, if you have a robot able to climb stairs and you detect stairs during navigation, you can then call another plugin for stair-climbing and afterwards transition back to normal navigation.

## Tutorials

The following section displays some examples and tutorials on how to use the statemachine, starting with the required setup to use the statemachine. Afterwards, an example launching the statemachine is presented and then a tutorial on writing and including your own plugin state. For an example of a plugin state implementation, see the [statemachine additions package](../statemachine_additions).

### Set up a robot for use with statemachine

Setting up a robot for the basic statemachine usage is fairly straightforward since it only requires setting up a robot motor controller interface that subscribes to command velocity messages of type [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html) and generates actual motor commands from them.

A service provider to tell the **Boot State** that the boot is finished is also required. This should ideally check if all necessary systems on your robot are up and running. The service provider needs to offer a service of type [std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html) under the name "statemachine/bootUpFinished". The following code snippet shows a rudimentary sample implementation in a node:

```cpp
#include "ros/ros.h"
#include "std_srvs/SetBool.h"

bool boot_finished = false;

bool bootUpService(std_srvs::SetBool::Request &req,
		std_srvs::SetBool::Response &res) {
	if (boot_finished) {
		res.success = 1;
		res.message = "Finished";
	} else {
		res.success = false;
		res.message = "Still booting ...";
	}
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "bootUpNode");
	ros::NodeHandle nh("statemachine");
	ros::ServiceServer bootup_service = nh.advertiseService("bootUpFinished",
			bootUpService);
	//checking boot process and setting boot_finished to true if finished
	ros::spin();
	return 0;
}
```

If this is not possible or necessary for your configuration, you can just launch the `bootUpNode` from the [statemachine additions package](../statemachine_additions) that sets up the service provider and returns a successful boot message after default 1 second. The delay can be set using the parameter `wait_time`. 

The setup for navigating to set goals and executing mapping behaviors or routines depends on the defined plugins and can therefore not generally be declared.

*Note*: If you plan on using the plugins for [ROS navigation](http://wiki.ros.org/navigation) provided in the [statemachine additions package](../statemachine_additions), you need to follow the [navigation stack robot setup tutorial](http://wiki.ros.org/navigation/Tutorials/RobotSetup).

In general, a tool for navigation, a tool for mapping and a tool for exploration is necessary to fully exploit the robot statemachine.

### Run statemachine

The statemachine's core functionality is distributed over several nodes that can simply be started with the launchfile `statemachine.launch` which requires the following arguments:
* `update_frequency`: The update rate in Hz of the statemachine (default: 20)
* `robot_frame`: The robot base frame (default: "base_footprint")
* `mapping_plugin`: The plugin used for mapping (default: "statemachine::MappingState")
* `calculate_goal_plugin`: The plugin used to calculate the next goal for exploration	 (default: "statemachine::CalculateGoalState")
* `navigation_plugin`: The plugin used for navigation (default: "statemachine::NavigationState")
* `autonomy_cmd_vel_topic`: The name of the command velocity topic for messages from exploration, waypoint following or simple goals (default: "/autonomy/cmd_vel")
* `teleoperation_cmd_vel_topic`:	The name of the command velocity topic for messages from teleoperation (default: "/teleoperation/cmd_vel")
* `cmd_vel_topic`: The name of the command velocity topic that the motor controller interface subscribes to (default: "/cmd_vel)
* `teleoperation_idle_timer`: Time in seconds without input from teleoperation that leads to a transition to **Idle State** (default: 0.5)
* `waypoint_routines`: List of all plugins to be used as routines for waypoints (default: [])
* `exploration_goal_tolerance`: Distance in all directions in meters that the robot's current position can differ from an exploration goal to still count it as reached (default: 0.05)

*Note*: The default plugins mentioned above all exist in the statemachine additions package.

The nodes can of course be started separately though it is easier to use the launch file. 

### Writing a plugin state

To create a plugin state to be used with the robot statemachine follow the upcoming steps. This is very similar to the ROS tutorial [Writing and Using a Simple Plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin) but also includes some specific details for the statemachine.

In your package, add the following code to the respective files:   

*CMakeLists.txt:*

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  statemachine
  statemachine_msgs
  ...
)
```

*package.xml:*

```xml
...
<build_depend>pluginlib</build_depend>
<build_export_depend>pluginlib</build_export_depend>
<exec_depend>pluginlib</exec_depend>
<exec_depend>statemachine</exec_depend>
<build_depend>statemachine</build_depend>
<build_export_depend>statemachine</build_export_depend>
<build_depend>statemachine_msgs</build_depend>
<build_export_depend>statemachine_msgs</build_export_depend>
<exec_depend>statemachine_msgs</exec_depend>
...
```

This adds all dependencies needed to use the [pluginlib](http://wiki.ros.org/pluginlib) and include the [Base State](#base-state).  
Next, create a class consisting of a header and source file in the respective directory in your package. The class needs to inherit from the [Base State](#base-state), interact with the [State Interface](#state-interface) and declare it is a plugin. The code for header and source are shown below.

*ExampleState.h*:

```cpp
#include <pluginlib/class_list_macros.h>
#include <statemachine/BaseState.h>
#include <statemachine/StateInterface.h>

namespace statemachine {

class ExampleState: public BaseState {

public:
	ExampleState();
	~ExampleState();
	void onSetup();
	void onEntry();
	void onActive();
	void onExit();
	void onExplorationStart(bool &success, std::string &message);
	void onExplorationStop(bool &success, std::string &message);
	void onWaypointFollowingStart(bool &success, std::string &message);
	void onWaypointFollowingStop(bool &success, std::string &message);
	void onInterrupt(int interrupt);
};

}
```

*ExampleState.cpp*:

```cpp
#include "ExampleState.h"

namespace statemachine {

ExampleState::ExampleState() {
	//...
}

ExampleState::~ExampleState() {
	//...
}

void ExampleState::onSetup() {
	//...
}

void ExampleState::onEntry() {
	//...
}

void ExampleState::onActive() {
	//...
}

void ExampleState::onExit() {
	//...
}

void ExampleState::onExplorationStart(bool &success,
		std::string &message) {
	//...
}

void ExampleState::onExplorationStop(bool &success,
		std::string &message) {
	//...
}

void ExampleState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	//...
}

void ExampleState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	//...
}

void ExampleState::onInterrupt(int interrupt) {
	//...
}

}

PLUGINLIB_EXPORT_CLASS(statemachine::ExampleState,
		statemachine::BaseState)

```

The state plugin needs to implement all methods declared in the [Base State](#base-state) as `virtual` and enables to add arbitrary functionality to them. The `PLUGINLIB_EXPORT_CLASS` macro registers the class as a plugin to the pluginlib.

To make the plugin available to ROS, an XML file needs to be added in the package that declares them as a library. The file should look like this:  
*statemachine_example_plugins.xml*:

```xml
<library path="lib/libstatemachine_example_plugins">
	<class type="statemachine::ExampleState"
		base_class_type="statemachine::BaseState">
		<description>This is the example state.</description>
	</class>
	...
</library>
```

It can feature multiple classes to declare in the same manner.  
The plugin library needs to be exported as well. Therefore the following lines need to be added to the *package.xml*:

```xml
<export>
	<statemachine plugin="${prefix}/statemachine_example_plugins.xml" />
</export>
```

*Note:* There can only be one `export` bracket in each *package.xml*.

With the following statement you can check in the terminal if the plugin was registered correctly:

    rospack plugins --attrib=plugin statemachine
    
It should show:

```
"your_package_name" /"your_workspace_path"/src/"your_package_name"/statemachine_example_plugins.xml
statemachine_additions /home/marco/catkin_ws/src/robot_statemachine/statemachine_additions/statemachine_plugins.xml
```
You can now use the plugin state in the robot statemachine.

### Use plugin state in the statemachine



### GUI introduction

## Nodes