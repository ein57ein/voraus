/** \file module_management_base.hpp
 * Basic functions for the module management in the \link voraus::Voraus
 * Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file includes the \link module_management::ModManBase ModManBase 
 * \endlink class. It depends on \link voraus::Voraus Voraus \endlink 
 * and robot_registration.srv (robot_registration.h)
 **/
/** \file robot_registration.srv
 * Config file for a ros service to register a robot at the master node.
 * Author : Martin Seidel
 * 
 * Catkin use this file to generate the robot_registration.h
 **/
#ifndef MODULE_MANAGEMENT_BASE_H_
#define MODULE_MANAGEMENT_BASE_H_

#include "voraus/voraus_base.hpp"
#include "voraus/robot_registration.h"

/** All components for the Module Management in the VORAUS framework
 */
namespace module_management
{
	/** Base class for the module management nodes.
	 * It provides a function for the registration of modules.
	 */
	class ModManBase : public voraus::Voraus
	{
		ros::ServiceServer module_registration;	/**< a service for module regestration on this client or server */ 

	protected:

		struct {
			bool task_scheduling;	/**< true if task for all robots will be scheduled on the server **/
			bool localisation;	/**< true if the localisation for all robots will be done on the server **/
			bool path_planing;	/**< true if pathes for all robots will be planed on the server **/
			bool demo;
		} central;	/**< contains the module configurations **/

		/** Initialize a instance of Voraus and register the service for
		 * \link #module_registration the registration of the modules 
		 * \endlink . Set #module to module manager.
		 * @param roshandle a valid ros::NodeHandle
		 * @param onServer indicates if the node runs on a server
		 */
		ModManBase(ros::NodeHandle roshandle, bool onServer)
		{
			module.module = module_modMan;
			module.type = "VORAUS modMan";
		
			initializeVoraus(roshandle, onServer, "modMan");
			
			module_registration = node.rosHandle.advertiseService("module_registration", &ModManBase::moduleLogOn, this);
		}

		/** Intentionally left empty
		 */
		virtual ~ModManBase() {}

	private:

		/** Callback for the module registration.
		 * @param req information from the requesting module
		 * @param res information for the requesting module
		 * @return true if the registration was successful
		 */
		bool moduleLogOn(voraus::module_registration::Request &req, voraus::module_registration::Response &res);
		
		/** This loop just calls loop().
		 * @param event includes the current and the last time stamps this function was called.
		 * @return error code
		 */
		int vorausLoop(const ros::TimerEvent& event) { return loop(event); }

	protected:

		/** This function provides a main loop for all inheriting classes.
		 * @param event includes the current and the last time stamps baseLoop() was called.
		 */
		virtual int loop(const ros::TimerEvent& event) {return 0;}
	};

	bool ModManBase::moduleLogOn(voraus::module_registration::Request &req, voraus::module_registration::Response &res)
	{
		std::string module = "";
		res.module_central = false;
					
		if (req.sending_module == module_undefind)
		{
			ROS_INFO("%s register a undefind Module. Please set the module value in the constructor of the sending Module.", name.full.c_str());
			return false;
		} else if (req.sending_module == module_modMan)
		{
			return false;
		} else if (req.sending_module == module_taSche)
		{
			module = "Task Scheduling";
			res.module_central = central.task_scheduling;
		} else if (req.sending_module == module_local)
		{
			module = "Localisation";
			res.module_central = central.localisation;
		} else if (req.sending_module == module_paPlan)
		{
			module = "Path Planing";
			res.module_central = central.path_planing;
		} else if (req.sending_module == module_movBase)
		{
			module = "Move Base";
		} else if (req.sending_module == module_maniCon)
		{
			module = "Manipulator Controller";
		} else if (req.sending_module == module_fluidCon)
		{
			module = "Fluid spout Controller";
		} else if (req.sending_module == module_demo)
		{
			module = "VORAUS Demo";
			res.module_central = central.demo;
		} else if (req.sending_module == module_misc)
		{
			module = "misc";			
		} else {
			ROS_WARN("%s a module with a invalid identificator (%i, type: %s) trys to register", name.full.c_str(), req.sending_module, req.type.c_str());
			return false;
		}
		
		res.id = name.id;
		if (name.id != 0) { ROS_INFO("%s register a %s Module with Type \"%s\"", name.full.c_str(), module.c_str(), req.type.c_str()); }
		return true;		
	}
};

#endif /*MODULE_MANAGEMENT_BASE_H_*/
