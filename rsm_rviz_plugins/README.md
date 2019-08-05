# RSM RViz Additions

Implements a RSM Control Panel, that can be added as a panel for RViz. See the [RSM documentation](../rsm_core#gui-introduction) for a detailed description. Also includes the **Plant Waypoint Tool** for RViz, that is explained in the formerly mentioned description as well. Futhermore, a node visualizing the waypoints as [interactive markers](http://wiki.ros.org/interactive_markers) is in the package.

## Documentation

The [Waypoint Following Visualization](#waypoint-following-visualization) realizes the visualiation of all waypoints as [interactive markers](http://wiki.ros.org/interactive_markers) in RViz.

### Waypoint Following Visualization

This class subscribes to the list of waypoints published by the [RSM package](../rsm_core#rsm_core) and visualizes them as [interactive markers](http://wiki.ros.org/interactive_markers). For each waypoint a flagpole with the waypoint's number overhead is shown. The waypoints can be moved on the x-y-plane by dragging them around, clicking on the surrounding circle or on the z-axis by dragging them at the arrows pointing up or down. Right clicking on the flagpole opens a menu, that shows the options to delete the waypoint or assign a routine.

All changes made to the waypoint in RViz are immediately forwarded to the [RSM data handler](../rsm_core#service-provider). The markers are only redrawn when one of the following changes occurs in the waypoint array:
* Waypoint list size changed
* Waypoint routine changed
* Waypoint visited status changed
* Waypoint unreachable status changed

When moving the waypoints, the visualization is not reloaded depending on the waypoint list because of the high data transfer this causes. This can lead to differences in the shown position of the marker and the actual position of the waypoint in the list when the system is experiencing high load and/or a slow connection. Therefore, there is a timer initiating a periodical refresh upon receiving the waypoints.

## Nodes

### waypointFollowingVisualizationNode

Visualizes all waypoints as interactive markers for RViz.

#### Subscribed Topics

**waypoints** ([rsm_msgs/WaypointArray](../rsm_msgs/msg/WaypointArray.msg))  
List of all waypoints and their information