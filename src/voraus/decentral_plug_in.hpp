/** \file decentral_plug_in.hpp
 * Contains the \link voraus::PlugInDecentral PlugInDecentral \endlink
 * class.
 * Author : Martin Seidel
 *
 * This file includes the \link voraus::PlugInBase PlugInBase \endlink
 * class. It depends on plug_in_base.hpp
 **/
#ifndef DECENTRAL_PLUG_IN_H_
#define DECENTRAL_PLUG_IN_H_

#include <voraus/plug_in_base.hpp>

namespace voraus
{
	/** This class contains basic functions for plugIns started on a
	 * decentral node (robot). If the module is in decentral mode a
	 * nodelet will be created for this robot.
	**/
	class PlugInDecentral : public PlugInBase {

		struct {
			boost::shared_ptr<nodelet::Nodelet> node; /**< a pointer to a Nodelet wihch starts the matching distributed part of the plugIn */
			robot_states state;	/**< the state of this robot */
			bool initialized;	/**< will be set to true if this node get a id */
		} robot;	/**< relevant informations for this %robot */

	public:
		/** Initalize variables
		* @param roshandle a nodehandle from roscore.
		* @param module_log_name this indicates the logging module. mostly between 4 and 7 characters
		* @param plug_in_name the full name (mostly the concatenation of the cpp namespace of the plugIn class and the class name, seperated by "::") of the plugIn, which contains the main functions of this node 
		**/
		PlugInDecentral(ros::NodeHandle roshandle, std::string module_log_name, std::string plug_in_name);
		
	private:
		/** Initialize a nodelet if the plugIn (module) runs in
		 * decentral mode
		 * @param event contain different times related to the calling timer
		 * @return error code
		 */
		int plugInLoop(const ros::TimerEvent& event);

	};
};

#endif /*DECENTRAL_PLUG_IN_H_*/
