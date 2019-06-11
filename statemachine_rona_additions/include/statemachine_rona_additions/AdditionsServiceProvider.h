#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

namespace statemachine {

/**
 * @class AdditionsServiceProvider
 * @brief Class that establishes communication between the different states and the statemachine's
 * 		  periphery including the GUI for the states and classes defined in statemachine Rona additions package. It offers
 * 		  services and publishes topics based on the variables that need to be saved during state transitions.
 */
class AdditionsServiceProvider {

public:
	AdditionsServiceProvider();
	~AdditionsServiceProvider();

private:
	ros::NodeHandle _nh;

	void publishTopics();
	ros::ServiceServer _set_navigation_to_reverse_service;

	ros::ServiceClient _set_rona_reverse_on;
	ros::ServiceClient _set_rona_reverse_off;

	bool setNavigationToReverse(std_srvs::SetBool::Request &req,
			std_srvs::SetBool::Response &res);
	/**
	 * Set Rona to reverse mode on or off depending on the parameter
	 * @param on If Rona should be set to on or off
	 * @return Returns true if the mode was set successfully, false otherwise
	 */
	bool setReverseModeRona(bool on);
};

}
