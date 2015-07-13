/** \file plug_in_definition.hpp
 * Contain the base class for Voraus PlugIns.
 * Author : Martin Seidel
 *
 * This file includes the \link voraus::PlugInDefinition 
 * PlugInDefinition \endlink class and depends on \link voraus::Voraus 
 * Voraus \endlink .
 **/
#ifndef PLUG_IN_DEFINITION_H_
#define PLUG_IN_DEFINITION_H_

#include "voraus/voraus_base.hpp"

namespace voraus
{
	/**	This class contain basic functions for Voraus plugIns. There are
	 * three parts, one for each \"execution context\" (central, 
	 * distributed = nodelet, decentral). Each part has a function for 
	 * initialization and two loop functions (one if the module is in 
	 * central mode and the other if the module is in decentral mode).
	 */
	class PlugInDefinition : public Voraus
	{
	protected:
		
		bool plugin_init;	/**< indicates if the initalization were done */
		bool module_in_central_mode;	/**< true if the module is running in a central mode */

	public:
		/** In what type of node was this plugIn started */
		enum excecute_on {
			central,	/**< plugIn is started inside a central node */
			nodelet,	/**< plugIn is started inside a nodelet */
			decentral	/**< plugIn is started inside a decentral node */
		};

		/** Initialized a instances of Voraus and call startTimer().
		 * @param roshandle a valide ros nodehandle
		 * @param project_namespace the namespace of the node, which starts this plugIn
		 * @param id the id of the node, which starts this plugIn
		 * @param name the name of the node, which starts this plugIn (server or roboter name)
		 * @param moduleCentral indicates if this module runs in central or decentral mode
		 * @param type where was this plugIn initialized
		 * @param module_log_name the name, which will be printed in front of log-mesages from this node
		 * @return error code
		 **/
		int initialize(ros::NodeHandle roshandle, std::string project_namespace, unsigned int id, std::string name, bool moduleCentral, excecute_on type, std::string module_log_name)
		{
			std::ostringstream log_text;
			log_text << module_log_name << "] [" << ((type == nodelet) ? "nodelet_" : "") << "plugIn";
			float loopRate = initializeVoraus(roshandle, (type == central) ? true : false, log_text.str().c_str(), name, project_namespace, id, true);
			module_in_central_mode = moduleCentral;

			if ( startTimer(type, loopRate) ) {return 1;}

			std::string temp_name = this->name.full;
			buildFullName();
			ROS_INFO("%s init PlugIn in %s mode", this->name.full.c_str(), (module_in_central_mode) ? "central" : "decentral");
			this->name.full = temp_name;
			
			plugin_init = true;
			return 0;
		}

		/** Return the state of initialition of this plugIn.
		 * @return true if this plugIn is initialized
		 */
		bool isInitialized() { return plugin_init; }
		
		/** Return the current mode (central or decentral) of this plugIn.
		 * @return true if this plugIn runs in central mode
		 */
		 bool isInCentralMode() { return module_in_central_mode; }

	private:
		/** Execute the Init functions and one ROS Timer. Which Timer 
		 * and Init function is executed depends on what type of node 
		 * the plugIn is started and in which mode the module is running.
		 * @param type the type of the node, this plugIn is running in
		 * @param loopRate [Hz] frequency for the started ros Timer
		 * @return error code
		 */
		int startTimer(excecute_on type, float loopRate)
		{
			float paramLoopRate = loopRate;
			
			if (type == central)
			{
				onInitCentral(&loopRate);
				mainLoopTimer = node.rosHandle.createTimer(ros::Rate(loopRate), &PlugInDefinition::centralPeriodicCallback, this);
			} else if (type == nodelet)
			{
				onInitNodelet(&loopRate);
				mainLoopTimer = node.rosHandle.createTimer(ros::Rate(loopRate), &PlugInDefinition::nodeletPeriodicCallback, this);
			} else if (type == decentral)
			{
				onInitDecentral(&loopRate);
				mainLoopTimer = node.rosHandle.createTimer(ros::Rate(loopRate), &PlugInDefinition::decentralPeriodicCallback, this);
			} else {
				ROS_ERROR("%s initialization off the PlugIn is impossible, because no valid type was given.", name.full.c_str());
				return 1;
			}

			if ( fabs(paramLoopRate - loopRate) > 0.1 ) { ROS_INFO("%s loopRate: %.2fHz", name.full.c_str(), loopRate); }

			return 0;
		}

	protected:
		/** Initialize #plugin_init */
		PlugInDefinition() {plugin_init = false;}

		/** Will be executed once if running inside a central node.
		 * @param loopTime [s] time for one Loop of the Callbacks in the central part
		 * @return error code
		 */
		virtual int onInitCentral(float *loopTime) {
			ROS_DEBUG("%s function \"onInitCentral\" is empty.", name.full.c_str());
			return 0;
		}

		/** Will be executed once if running inside a nodelet.
		 * @param loopTime [s] time for one Loop of the Callbacks in the nodelet part
		 * @return error code
		 */
		virtual int onInitNodelet(float *loopTime) {
			ROS_DEBUG("%s function \"onInitNodelet\" is empty.", name.full.c_str());
			return 0;
		}

		/** Will be executed once if running inside a decentral node.
		 * @param loopTime [s] time for one Loop of the Callbacks in the decentral part
		 * @return error code
		 */
		virtual int onInitDecentral(float *loopTime) {
			ROS_DEBUG("%s function \"onInitDecentral\" is empty.", name.full.c_str());
			return 0;
		}

		/** The main periodic event for the central part of the flexible
		 * module.
		 * @param event includes the current and the last time stamps this function was called.
		 */
		virtual void centralPeriodicCallback(const ros::TimerEvent& event) = 0;

		/** The main periodic event for the nodelet part of the flexible
		 * module.
		 * @param event includes the current and the last time stamps this function was called.
		 */
		virtual void nodeletPeriodicCallback(const ros::TimerEvent& event) = 0;

		/** The main periodic event for the decentral part of the flexible
		 * module.
		 * @param event includes the current and the last time stamps this function was called.
		 */
		virtual void decentralPeriodicCallback(const ros::TimerEvent& event) = 0;

	public:
		/** Intentionally left empty
		 */
		virtual ~PlugInDefinition(){}
	};
};

#endif //PLUG_IN_DEFINITION_H_
