# Statemachine Rona Additions

Navigation for the robot statemachine using the [Rona navigation package](https://github.com/schmiddey/rona) which is available as open-source. Includes a navigation plugin and an additional data handler.

## Documentation

This package implements a [navigation plugin](#rona-navigation-state) that interfaces the [Rona navigation package](https://github.com/schmiddey/rona) and an [additional data handler class](#additions-service-provider).

### Rona Navigation State

This state plugin interfaces the [Rona navigation package](https://github.com/schmiddey/rona). Received goals are forwarded to it and feedback received from it regarding the progress. If it fails, the goal is added to the failed goals list. If it succeeds, the failed goal list will be reset.

When standing still for too long, it transitions to the [Idle State](../statemachine#non-customizable-states).
Reaching the goal will initiate a transition to the respective **Mapping State** or the particular routine state if there is one available. If not, [Waypoint Following State](../statemachine#non-customizable-states) is called. After reaching a navigation goal provided by RViz and if waypoint following has ended, it transitions to [Idle State](../statemachine#non-customizable-states).

Reverse mode is handled internally in [Rona navigation](https://github.com/schmiddey/rona) and therefore no additional logic was implemented in this state.

### Additions Service Provider

For driving in reverse mode, it provides a service that is called when reverse mode should be activated. This service calls a service provided by [Rona navigation](https://github.com/schmiddey/rona) that switches between forward and backwards driving.

## Nodes

### additionsServiceProviderNode

An additional data handler class that adds services to interface the Rona navigation package.

#### Services

**setNavigationToReverse** ([std_srvs/SetBool](http://docs.ros.org/api/std_srvs/html/srv/SetBool.html))  
Needs to be implemented for reverse mode, just returns success
