/** \file decentral_plug_in_controller.cpp
 * Create a instance of a decentral plug in controller from the \link 
 * voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file depends on decentral_plug_in.hpp
 **/

#include "voraus/decentral_plug_in.hpp"

/** Create a ROS NodeHandle. Create a Instance of the \link
 * voraus#PlugInDecentral decentral plug in controller \endlink . Excute
 * ros::spin().
 */
int main(int argc, char **argv)
{
	ros::init(argc, argv, "decentral_plugIn_controller");
	ros::NodeHandle roshandle;

	std::string module_log_name, plug_in_name;

	ros::param::param<std::string>("~plug_in_name", plug_in_name, "");
	ros::param::param<std::string>("~log_name", module_log_name, "");
	
	voraus::PlugInDecentral *plugIn;

	if (module_log_name != "")
	{
		plugIn = new voraus::PlugInDecentral(roshandle, module_log_name, plug_in_name);
		ros::spin();
	} else {
		ROS_ERROR("\"~log_name\" is not set. It will be needed for log messages to indicate this node.");
	}

	return 0;
}
