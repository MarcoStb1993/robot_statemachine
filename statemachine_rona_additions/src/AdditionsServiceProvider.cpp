#include <statemachine_rona_additions/AdditionsServiceProvider.h>

namespace statemachine {

AdditionsServiceProvider::AdditionsServiceProvider() {

	ros::NodeHandle nh("statemachine");

	_set_navigation_to_reverse_service = nh.advertiseService(
			"setNavigationToReverse",
			&AdditionsServiceProvider::setNavigationToReverse, this);

	_set_rona_reverse_on = _nh.serviceClient<std_srvs::Empty>(
			"rona/move/set_reverse_on");
	_set_rona_reverse_off = _nh.serviceClient<std_srvs::Empty>(
			"rona/move/set_reverse_off");
}

AdditionsServiceProvider::~AdditionsServiceProvider() {

}

bool AdditionsServiceProvider::setNavigationToReverse(
		std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
	if (setReverseModeRona(req.data)) {
		res.success = 1;
		res.message = "Mode set";
	} else {
		res.success = 0;
		res.message = "Unable to set Mode in Rona";
	}
	return true;
}

bool AdditionsServiceProvider::setReverseModeRona(bool on) {
	std_srvs::Empty srv;
	if (on) {
		if (_set_rona_reverse_on.call(srv)) {
			return true;
		} else {
			ROS_ERROR("Failed to call Set Reverse Mode On service");
			return false;
		}
	} else {
		if (_set_rona_reverse_off.call(srv)) {
			return true;
		} else {
			ROS_ERROR("Failed to call Set Reverse Mode Off service");
			return false;
		}
	}
}

}
