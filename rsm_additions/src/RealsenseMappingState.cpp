#include <rsm_additions/RealsenseMappingState.h>

namespace rsm {

RealsenseMappingState::RealsenseMappingState() {
}

RealsenseMappingState::~RealsenseMappingState() {
}

void RealsenseMappingState::onSetup() {
	_joint_states_subscriber = _nh.subscribe("joint_states", 10,
			&RealsenseMappingState::jointStateCallback, this);
	_realsense_joint_controller = _nh.advertise<std_msgs::Float64>(
			"realsense_controller/command", 1, true);
	ros::NodeHandle nh("rsm");
	_reset_realsense_position_client = nh.serviceClient<std_srvs::Trigger>(
			"resetRealsensePosition");
	_navigation_goal_completed_service = nh.serviceClient<
			rsm_msgs::GoalCompleted>("navigationGoalCompleted");
	_name = "E: RealSense Mapping";
	_swivel_state = -1;
	_position_reached = false;
	_message_send = false;
	_navigation_completed_status = rsm_msgs::GoalStatus::ABORTED;
}

void RealsenseMappingState::onEntry() {
	_swivel_state = MOVE_LEFT;
}

void RealsenseMappingState::onActive() {
	switch (_swivel_state) {
	case MOVE_LEFT: {
		if (_position_reached) {
			_swivel_state = MOVE_RIGHT;
			_position_reached = false;
			_message_send = false;
		} else {
			if (!_message_send) {
				std_msgs::Float64 realsense_command;
				realsense_command.data = REALSENSE_LEFT_LIMIT;
				_realsense_joint_controller.publish(realsense_command);
				ros::spinOnce();
				_message_send = true;
			}
		}
		break;
	}
	case MOVE_RIGHT: {
		if (_position_reached) {
			_swivel_state = MOVE_TO_CENTER;
			_position_reached = false;
			_message_send = false;
		} else {
			if (!_message_send) {
				std_msgs::Float64 realsense_command;
				realsense_command.data = REALSENSE_RIGHT_LIMIT;
				_realsense_joint_controller.publish(realsense_command);
				_message_send = true;
			}
		}
		break;
	}
	case MOVE_TO_CENTER: {
		if (_position_reached) {
			if (!_interrupt_occured) {
				_navigation_completed_status = rsm_msgs::GoalStatus::REACHED;
				_stateinterface->transitionToVolatileState(
						_stateinterface->getPluginState(CALCULATEGOAL_STATE));
			}
		} else {
			if (!_message_send) {
				std_msgs::Float64 realsense_command;
				realsense_command.data = REALSENSE_CENTER_POSITION;
				_realsense_joint_controller.publish(realsense_command);
				_message_send = true;
			}
		}
		break;
	}
	default: {
		break;
	}
	}
}

void RealsenseMappingState::onExit() {
	if (!_position_reached) {
		std_srvs::Trigger srv;
		if (!_reset_realsense_position_client.call(srv)) {
			ROS_ERROR("Failed to call Reset Realsense Position service");
		}
	}
	rsm_msgs::GoalCompleted srv;
	srv.request.status.goal_status = _navigation_completed_status;
	if (!_navigation_goal_completed_service.call(srv)) {
		ROS_ERROR("Failed to call Complete Navigation Goal service");
	}
}

void RealsenseMappingState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void RealsenseMappingState::onExplorationStop(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void RealsenseMappingState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void RealsenseMappingState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void RealsenseMappingState::onInterrupt(int interrupt) {
	switch (interrupt) {
	case EMERGENCY_STOP_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<EmergencyStopState>());
		_interrupt_occured = true;
		break;
	case TELEOPERATION_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				boost::make_shared<TeleoperationState>());
		_interrupt_occured = true;
		break;
	case SIMPLE_GOAL_INTERRUPT:
		_stateinterface->transitionToVolatileState(
				_stateinterface->getPluginState(NAVIGATION_STATE));
		_interrupt_occured = true;
		break;
	}
}

void RealsenseMappingState::jointStateCallback(
		sensor_msgs::JointState::ConstPtr joint_state) {
	switch (_swivel_state) {
	case MOVE_LEFT: {
		if (joint_state->position[0] >= REALSENSE_LEFT_LIMIT - POS_TOLERANCE) {
			_position_reached = true;
		}
		break;
	}
	case MOVE_RIGHT: {
		if (joint_state->position[0] <= REALSENSE_RIGHT_LIMIT + POS_TOLERANCE) {
			_position_reached = true;
		}
		break;
	}
	case MOVE_TO_CENTER: {
		if (joint_state->position[0] >= REALSENSE_CENTER_POSITION - POS_TOLERANCE
				&& joint_state->position[0]
						<= REALSENSE_CENTER_POSITION + POS_TOLERANCE) {
			_position_reached = true;
		}
		break;
	}
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::RealsenseMappingState, rsm::BaseState)
