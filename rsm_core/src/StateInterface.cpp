#include <rsm_core/StateInterface.h>

namespace rsm {

StateInterface::StateInterface() :
		_plugin_loader("rsm_core", "rsm::BaseState") {
	ros::NodeHandle private_nh("~");
	private_nh.param<std::string>("calculate_goal_plugin",
			_calculate_goal_plugin, "rsm::CalculateGoalState");
	private_nh.param<std::string>("navigation_plugin", _navigation_plugin,
			"rsm::NavigationState");
	private_nh.param<std::string>("mapping_plugin", _mapping_plugin,
			"rsm::MappingDummyState");

	ros::NodeHandle nh("rsm");
	_operation_mode_sub = nh.subscribe<rsm_msgs::OperationMode>(
			"operationMode", 1, &StateInterface::operationModeCallback, this);
	_simple_goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("simpleGoal", 1,
			&StateInterface::simpleGoalCallback, this);
	_start_stop_exploration_service = nh.advertiseService(
			"startStopExploration",
			&StateInterface::startStopExplorationService, this);
	_start_stop_waypoint_following_service = nh.advertiseService(
			"startStopWaypointFollowing",
			&StateInterface::startStopWaypointFollowingService, this);
	_stop_2d_nav_goal_service = nh.advertiseService("stop2DNavGoal",
			&StateInterface::stop2dNavGoal, this);
	_state_info_publisher = nh.advertise<std_msgs::String>("stateInfo", 10);
	_state_info_service = nh.advertiseService("stateInfo",
			&StateInterface::stateInfoService, this);
	_set_navigation_goal_client = nh.serviceClient<
			rsm_msgs::SetNavigationGoal>("setNavigationGoal");

	_current_state = NULL;
	_next_state = NULL;
	_on_interrupt = false;
}

StateInterface::~StateInterface() {
	if (_current_state) {
		_current_state->onExit();
	}
}

boost::shared_ptr<rsm::BaseState> StateInterface::getPluginState(
		int plugin_type, std::string routine) {
	try {
		switch (plugin_type) {
		case 1: {
			return _plugin_loader.createInstance(_calculate_goal_plugin);
			break;
		}
		case 2: {
			return _plugin_loader.createInstance(_navigation_plugin);
			break;
		}
		case 3: {
			return _plugin_loader.createInstance(_mapping_plugin);
			break;
		}
		case 4: {
			std::ostringstream s;
			s << "rsm::" << routine << "RoutineState";
			return _plugin_loader.createInstance(s.str());
			break;
		}
		default: {
			if (!routine.empty()) {
				std::ostringstream s;
				s << "rsm::" << routine;
				return _plugin_loader.createInstance(s.str());
			} else {
				ROS_ERROR(
						"No matching plugin type found, return to Idle State");
				return boost::make_shared<IdleState>();
			}
			break;
		}
		}
	} catch (const std::exception& e) {
		ROS_ERROR("Plugin state could not be created, return to Idle State");
		return boost::make_shared<IdleState>();
	}
}

void StateInterface::awake() {
// check 1st time awake
	if (!_current_state && _next_state) {
		_current_state = _next_state;
		_current_state->onEntry();
		_next_state = NULL;
	}

	if (_current_state) {
		_current_state->onActive();

		if (_next_state) // do transition
		{
			// do de-initialization of current state
			_current_state->onExit();
		}
	}

	if (_next_state) // do transition
	{
		// do transition to next state
		_current_state = _next_state;
		_next_state = NULL;

		_current_state->onEntry();
		std_msgs::String state_info;
		state_info.data = _current_state->getName();
		_state_info_publisher.publish(state_info);
	}
}

void StateInterface::transitionToVolatileState(
		boost::shared_ptr<rsm::BaseState> nextState) {
	if (_current_state != nextState) {
		if (nextState != NULL && nextState->getStateInterface() == NULL) {
			_next_state = nextState;
			_next_state->setStateInterface(this);
			_next_state->onSetup();
		} else {
			ROS_ERROR(
					"Next state instance invalid. Either NULL or already assigned state passed.");
		}
	}
}

void StateInterface::operationModeCallback(
		const rsm_msgs::OperationMode::ConstPtr& operation_mode) {
	if (operation_mode->emergencyStop) {
		_on_interrupt = true;
		if (_current_state) {
			_current_state->onInterrupt(EMERGENCY_STOP_INTERRUPT);
		}
	} else if (operation_mode->mode == 2) {
		_on_interrupt = true;
		if (_current_state) {
			_current_state->onInterrupt(TELEOPERATION_INTERRUPT);
		}
	} else {
		if (_on_interrupt) {
			_on_interrupt = false;
			if (_current_state) {
				_current_state->onInterrupt(INTERRUPT_END);
			}
		}
	}
}

void StateInterface::simpleGoalCallback(
		const geometry_msgs::PoseStamped::ConstPtr& goal) {
	_on_interrupt = true;
	if (_current_state) {
		_current_state->onInterrupt(SIMPLE_GOAL_INTERRUPT);
	}
	rsm_msgs::SetNavigationGoal srv;
	srv.request.goal = goal->pose;
	srv.request.navigationMode = SIMPLE_GOAL;
	if (_set_navigation_goal_client.call(srv)) {
	} else {
		ROS_ERROR("Failed to call Set Navigation Goal service");
	}
}

bool StateInterface::startStopExplorationService(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	if (_current_state) {
		bool success;
		std::string message;
		if (req.data) {
			_current_state->onExplorationStart(success, message);
		} else {
			_current_state->onExplorationStop(success, message);
		}
		res.success = success;
		res.message = message;
	} else {
		res.success = false;
		res.message = "No active state";
	}
	return true;
}

bool StateInterface::startStopWaypointFollowingService(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	if (_current_state) {
		bool success;
		std::string message;
		if (req.data) {
			_current_state->onWaypointFollowingStart(success, message);
		} else {
			_current_state->onWaypointFollowingStop(success, message);
		}
		res.success = success;
		res.message = message;
	} else {
		res.success = false;
		res.message = "No active state";
	}
	return true;
}

bool StateInterface::stop2dNavGoal(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	if(_current_state){
		_current_state->onInterrupt(SIMPLE_GOAL_STOP_INTERRUPT);
	}
	return true;
}

bool StateInterface::stateInfoService(std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res) {
	res.success = true;
	res.message = _current_state->getName();
	return true;
}

}
