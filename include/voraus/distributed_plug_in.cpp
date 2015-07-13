/** \file distributed_plug_in.cpp
 * Function(s) for the nodelet part of the plug-in controller in the 
 * \link voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file depends on distributed_plug_in.hpp
 **/
#include "distributed_plug_in.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(voraus::DistributedPlugIn, nodelet::Nodelet)

namespace voraus
{
	void DistributedPlugIn::onInit()
	{
		bool onServer = false, noMainLoopTimer = true;
		
		std::string robot_name = this->getName();
		std::string project_namespace = this->getMyArgv()[0];
		unsigned int robot_id = atoi(this->getMyArgv()[1].c_str());
		std::string plug_in_name = this->getMyArgv()[2];
		std::string module_log_name = this->getMyArgv()[3];
		bool module_central = (this->getMyArgv()[4] == "true") ? true : false;
		
		std::ostringstream module;
		module << module_log_name << "] [nodelet";
		
		initializeVoraus(this->getNodeHandle(), onServer, module.str().c_str(), robot_name.c_str(), project_namespace.c_str(), robot_id, noMainLoopTimer);
				
		NODELET_DEBUG("%s initialize plugIn nodelet for [%i %s] in namespace \"%s\". Waiting for a initialization.", name.full.c_str(), name.id, name.name.c_str(), name.project_namespace.c_str());

		bool plug_in_loaded = false;
		try
		{
			plug_in_node = plug_in_loader.createInstance(plug_in_name.c_str());
			plug_in_loaded = true;
		} catch(pluginlib::PluginlibException& ex)
		{
			NODELET_ERROR("%s failed to load \"%s\". Error: %s", name.full.c_str(), plug_in_name.c_str(), ex.what());
		}

		if (plug_in_loaded)
		{
			plug_in_node->initialize(node.rosHandle, name.project_namespace, name.id, name.name, module_central, voraus::PlugInDefinition::nodelet, module_log_name);
		}
	}
}
