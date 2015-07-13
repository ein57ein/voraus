/** \file central_plug_in.hpp
 * Contains the \link voraus::PlugInCentral PlugInCentral \endlink class.
 * Author : Martin Seidel
 *
 * This file includes the \link voraus::PlugInBase PlugInBase \endlink
 * class. It depends on plug_in_base.hpp
 **/
#ifndef CENTRAL_PLUG_IN_H_
#define CENTRAL_PLUG_IN_H_

#include <voraus/plug_in_base.hpp>

namespace voraus
{
	/** This class contains basic functions for plugIns started on a
	 * central node (server). It listen to the \link heartbeat.msg
	 * heartbeat messages \endlink from the \link module_management::ModManCentral
	 * central module manager \endlink . If the linked module is in
	 * central mode a nodelet will be created for every robot in the
	 * heartbeat list.	
	 */
	class PlugInCentral : public PlugInBase
	{
		ros::Subscriber serverHeartbeat;	/**< receive \link heartbeat.msg heartbeat messages \endlink from the \link module_management::ModManCentral central module manager \endlink */

		/** Informations about a known robot */
		struct robot_list_params {
			boost::shared_ptr<nodelet::Nodelet> node; /**< a pointer to a Nodelet wihch starts the matching distributed part of the plugIn */
			unsigned int id;	/**< the id of the robot */
			std::string name;	/**< the name of the robot */
			robot_states state;	/**< the state of the robot send with the last heartbeat */
			unsigned int lastHeartbeatWithRobotIdCounter;	/**< the last time a heartbeat message with the robot id was received */ 
		};

		std::vector<robot_list_params> robot_list;	/**< List of the known robots */

	public:
		/** Initalize the subscriber for the \link heartbeat.msg
		 * heartbeat messages \endlink from the \link module_management::ModManCentral
		 * central module manager \endlink
		 * @param roshandle a nodehandle from roscore.
		 * @param module_log_name this indicates the logging module. mostly between 4 and 7 characters
		 * @param plug_in_name the full name (mostly the concatenation of the cpp namespace of the plugIn class and the class name, seperated by "::") of the plugIn, which contains the main functions of this node 
		 */
		PlugInCentral(ros::NodeHandle roshandle, std::string module_log_name, std::string plug_in_name);
		
	private:

		/** Print a warning if a robot is lost and remove him.
		 * @param event contain different times related to the calling timer
		 * @return error code
		 */
		int plugInLoop(const ros::TimerEvent& event);

		/** Add new robots to the #robot_list , update known robots and
		 * remove robots, which id wasn't send for the last x heartbeats.
		 * x depends on the variable current_known_robot_count in the event
		 * @param event contain a entry from the list of known robot on the server
		 **/
		void serverRobotListCallback(const voraus::heartbeat event);

		/** Removes a robot from the PlugInCentral#robot_list if his 
		 * state is \" Voraus#lost \"
		 * @param dead_robot information about the robot, which should be checked
		 * @return if true the robot will be removed
		 */
		friend bool removeRobot(robot_list_params dead_robot);
	};
};

#endif /*CENTRAL_PLUG_IN_H_*/
