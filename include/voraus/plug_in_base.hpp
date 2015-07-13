/** \file plug_in_base.hpp
 * Basic functions for controlling plug-ins in the \link voraus::Voraus 
 * Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file includes the \link voraus::PlugInBase PlugInBase \endlink
 * class. It depends on \link voraus::Voraus Voraus \endlink and
 * plug_in_definition.hpp.
 **/
#ifndef PLUG_IN_BASE_H_
#define PLUG_IN_BASE_H_

#include "voraus/voraus_base.hpp"

#include <pluginlib/class_loader.h>
#include <nodelet/nodelet.h>

#include "voraus/plug_in_definition.hpp"

namespace voraus
{
	/** Base class for the plug-in controller. It starts a plug-In and 
	 * provide a function which starts a nodelet.
	 */
	class PlugInBase : public Voraus
	{
		pluginlib::ClassLoader<nodelet::Nodelet> nodelet_loader;	/**< a pluginLoader for a Nodelet */
		pluginlib::ClassLoader<voraus::PlugInDefinition> plug_in_loader;	/**< a pluginLoader for a definded Voraus plugIn **/
		
	protected:

		struct {
			boost::shared_ptr<voraus::PlugInDefinition> node;	/**< a pointer to a instances of a Voraus plugIn **/
			std::string name;	/**< the complete name of the plugIn. */
			std::string module_log_name;	/**< the name, which will be printed in front of log-mesages from this node **/
		} plugIn;	/**< contains all plugIn relevant information */

		/** Initialize an instance of Voraus and create an instance of 
		 * the plugIn.
		 * @param roshandle a nodehandle from roscore.
		 * @param onServer indicates if this node is a central one
		 * @param module_log_name the name, which will be printed in front of log-mesages from this node
		 * @param plug_in_name the name of the plugIn inside the ROS-PlugIn-System
		 */
		PlugInBase(ros::NodeHandle roshandle, bool onServer, std::string module_log_name, std::string plug_in_name);
		
		/** Intentionally left empty
		 */
		virtual ~PlugInBase() {}

	private:

		/** Start the plugIn if it's not initalized and call plugInLoop().
		 * @param event includes the current and the last time stamps this function was called.
		 * @return error code
		 */
		int vorausLoop(const ros::TimerEvent& event);

		/* Build-up the #name and starts a intances of a plugIn
		 * @return error code
		
		int startPlugIn(); */


	protected:
		/** Will be called from the vorausLoop(). Could be used by
		 * inheritied classes as loop.
		 * @param event includes the current and the last time stamps vorausLoop() was called with
		 * @return error code
		 */
		virtual int plugInLoop(const ros::TimerEvent& event) {return 0;}

		/** Create and initalize a nodelet with a \link 
		 * voraus::DistributedPlugIn DistributedPlugIn \endlink inside.
		 * @param node pointer to the nodelet which should be inialized
		 * @param bot_id id of the robot which will be represented by the created nodelet
		 * @param bot_name name of the robot which will be represented by the created nodelet
		 * @return error code
		 */
		int createNodelet(boost::shared_ptr<nodelet::Nodelet> *node, unsigned int bot_id, std::string bot_name);
	};

	PlugInBase::PlugInBase(ros::NodeHandle roshandle, bool onServer, std::string module_log_name, std::string plug_in_name):nodelet_loader("nodelet", "nodelet::Nodelet"), plug_in_loader("voraus","voraus::PlugInDefinition")
	{
		plugIn.module_log_name = module_log_name;
		plugIn.name = plug_in_name;
		
		initializeVoraus(roshandle, onServer, plugIn.module_log_name.c_str());

		ROS_INFO("%s plug_in_name: \"%s\"", name.full.c_str(), plugIn.name.c_str());
		
		if (plugIn.name == "") { THROW(voraus::VorausException::no_plug_in) }

		try
		{
			plugIn.node = plug_in_loader.createInstance(plugIn.name.c_str());
			module.module = plugIn.node->get_module();
			module.type = plugIn.node->get_type();
			ROS_DEBUG("%s load module \"%s\" with module_id %i", name.full.c_str(), module.type.c_str(), module.module);
		} catch(pluginlib::PluginlibException& ex)
		{
			ROS_ERROR("%s [plugIn] failed to load \"%s\". Error: %s", name.full.c_str(), plugIn.name.c_str(), ex.what());
		}		
	}

	int PlugInBase::vorausLoop(const ros::TimerEvent& event)
	{
		if (name.id != 0 && !plugIn.node->isInitialized()) {
			plugIn.node->initialize(node.rosHandle, name.project_namespace, name.id, name.name, module.central, (node.onServer) ? voraus::PlugInDefinition::central : voraus::PlugInDefinition::decentral, plugIn.module_log_name);
		}
		
		return plugInLoop(event);
	}

	int PlugInBase::createNodelet(boost::shared_ptr<nodelet::Nodelet> *node, unsigned int bot_id, std::string bot_name)
	{
		std::string nodeletName = "voraus::DistributedPlugIn";
		
		try
		{
			*node = nodelet_loader.createInstance(nodeletName.c_str());
		} catch(pluginlib::PluginlibException& ex)
		{
			ROS_ERROR("%s NodeletPlugIn: failed to load \"%s\" with id=%i and name=\"%s\". Error: %s", name.full.c_str(), nodeletName.c_str(), bot_id, bot_name.c_str(), ex.what());
			return 1;
		}

		std::map<std::string, std::string> remap;
		std::vector<std::string> param;
		param.push_back(name.project_namespace);
		param.push_back( static_cast<std::ostringstream*>( &(std::ostringstream() << bot_id) )->str() );
		param.push_back(plugIn.name);
		param.push_back(plugIn.module_log_name);
		param.push_back( (module.central)?"true":"false" );
		
		(*node)->init(bot_name, remap, param);

		return 0;
	}
};

#endif /*PLUG_IN_BASE_H_*/
