#include <rsm_additions/KinectMappingState.h>

namespace rsm {

KinectMappingState::KinectMappingState() {
}

KinectMappingState::~KinectMappingState() {
}

void KinectMappingState::onSetup() {
	_joint_states_subscriber = _nh.subscribe("joint_states", 10,
			&KinectMappingState::jointStateCallback, this);
	_kinect_joint_controller = _nh.advertise<std_msgs::Float64>(
			"kinetic_controller/command", 1, true);
	ros::NodeHandle nh("rsm");
	_reset_kinect_position_client = nh.serviceClient<std_srvs::Trigger>(
			"resetKinectPosition");
	_name = "Mapping";
	_swivel_state = -1;
	_position_reached = false;
	_message_send = false;
}

void KinectMappingState::onEntry() {
	_swivel_state = MOVE_LEFT;
}

void KinectMappingState::onActive() {
	switch (_swivel_state) {
	case MOVE_LEFT: {
		if (_position_reached) {
			_swivel_state = MOVE_RIGHT;
			_position_reached = false;
			_message_send = false;
		} else {
			if (!_message_send) {
				std_msgs::Float64 kinetic_command;
				kinetic_command.data = KINECT_LEFT_LIMIT;
				_kinect_joint_controller.publish(kinetic_command);
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
				std_msgs::Float64 kinetic_command;
				kinetic_command.data = KINECT_RIGHT_LIMIT;
				_kinect_joint_controller.publish(kinetic_command);
				_message_send = true;
			}
		}
		break;
	}
	case MOVE_TO_CENTER: {
		if (_position_reached) {
			if (!_interrupt_occured) {
				_stateinterface->transitionToVolatileState(
						_stateinterface->getPluginState(CALCULATEGOAL_STATE));
			}
		} else {
			if (!_message_send) {
				std_msgs::Float64 kinetic_command;
				kinetic_command.data = KINECT_CENTER_POSITION;
				_kinect_joint_controller.publish(kinetic_command);
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

void KinectMappingState::onExit() {
	if (!_position_reached) {
		std_srvs::Trigger srv;
		if (!_reset_kinect_position_client.call(srv)) {
			ROS_ERROR("Failed to call Reset Kinect Position service");
		}
	}
}

void KinectMappingState::onExplorationStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void KinectMappingState::onExplorationStop(bool &success,
		std::string &message) {
	success = true;
	message = "Exploration stopped";
	_stateinterface->transitionToVolatileState(boost::make_shared<IdleState>());
}

void KinectMappingState::onWaypointFollowingStart(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void KinectMappingState::onWaypointFollowingStop(bool &success,
		std::string &message) {
	success = false;
	message = "Exploration running";
}

void KinectMappingState::onInterrupt(int interrupt) {
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

void KinectMappingState::jointStateCallback(
		sensor_msgs::JointState::ConstPtr joint_state) {
	switch (_swivel_state) {
	case MOVE_LEFT: {
		if (joint_state->position[0] >= KINECT_LEFT_LIMIT - POS_TOLERANCE) {
			_position_reached = true;
		}
		break;
	}
	case MOVE_RIGHT: {
		if (joint_state->position[0] <= KINECT_RIGHT_LIMIT + POS_TOLERANCE) {
			_position_reached = true;
		}
		break;
	}
	case MOVE_TO_CENTER: {
		if (joint_state->position[0] >= KINECT_CENTER_POSITION - POS_TOLERANCE
				&& joint_state->position[0]
						<= KINECT_CENTER_POSITION + POS_TOLERANCE) {
			_position_reached = true;
		}
		break;
	}
	}
}

}

PLUGINLIB_EXPORT_CLASS(rsm::KinectMappingState,
		rsm::BaseState)
