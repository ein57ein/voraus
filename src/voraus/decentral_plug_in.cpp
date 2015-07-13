/** \file decentral_plug_in.cpp
 * Contains the functions of the \link voraus::PlugInDecentral 
 * PlugInDecentral \endlink class.
 * Author : Martin Seidel
 *
 * This file depends on decentral_plug_in.hpp
 **/
#include "decentral_plug_in.hpp"

namespace voraus
{
	PlugInDecentral::PlugInDecentral(ros::NodeHandle roshandle, std::string module_log_name, std::string plug_in_name):PlugInBase(roshandle, false, module_log_name, plug_in_name)
	{
		robot.initialized = false;
		robot.state = unregistered;
	}

	int PlugInDecentral::plugInLoop(const ros::TimerEvent& event)
	{
		if ( !robot.initialized && name.id > 1)
		{
			if (!module.central) { createNodelet(&(robot.node), name.id, name.name); }
			robot.initialized = true;
		}

		return 0;
	}
}
