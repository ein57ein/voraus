/** \file decentral_module_management.hpp
 * Contains the class definition of the decentral module managemener in 
 * the \link voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file includes the ModManDecentral class. It depends on
 * module_management_base.hpp
 **/
#ifndef DECENTRAL_MODULE_MANAGEMENT_H_
#define DECENTRAL_MODULE_MANAGEMENT_H_

#include "module_management/module_management_base.hpp"

namespace module_management
{
	/** This class receive the list of known robots from the \link 
	 * ModManCentral central module management \endlink as Heartbeats 
	 * and send state messages to it. It should be used on a decentral 
	 * node (usually a robot).
	 */
	//This state is a result of the internal robot states (f.e. from the task scheduling module).
	class ModManDecentral : public ModManBase {

		ros::ServiceClient logIn;	/**< first contact to server send name, get id and config **/
		ros::Subscriber serverHeartbeat;	/**< subscribs to the topic with server heartbeats **/
		ros::Publisher robotHeartbeat;	/**< publisher for the robot heartbeat **/
		
		struct {
			ros::Duration maxTimeWithoutHeartbeat;	/**< [s] if the server sends no heartbeats for this period the robot change into unregistered state **/
			ros::Time lastHeartbeatTime;	/**< last Time a heartbeat from the server arrived **/
			int lastHeartbeatWithMyIdCounter;	/**< a counter how many heartbeats was received without the id of this robot **/
			robot_states state;	/**< current state of the robot **/
		}robot;	/**< contains all robot specific informtions **/

	public:
		/** Initialize the robot state and the Publisher, Subscriber and
		 * ServiceClient 
		 *	@param roshandle a nodehandle from roscore.
		 **/
		ModManDecentral(ros::NodeHandle roshandle);
		
	private:
		
		/** If not unregistered, it calls the sendHeartbeat Function and
		 * checks if the last server heartbeat arrived in the allowed 
		 * time
		 * @param event contain different times related to the calling timer
		 * @return error code
		 **/
		int loop(const ros::TimerEvent& event);

		/** Sending a heartbeat message.
		 * 	@return error code
		 **/
		int sendHeartbeat();

		/** Calls the registration-service of the server-node. Set timeout,
		 *	id, state and the module config in any case.
		 * 	@return error code
		 **/
		int registerAtServer();

		/** Receives heartbeats from the server-node (list of known robots).
		 *  Proves if his own id was sent.
		 *	@param event contain a entry from the list of known robot on the server
		 **/
		void serverHeartbeatCallback(const voraus::heartbeat event);
	};
};

#endif /*DECENTRAL_MODULE_MANAGEMENT_H_*/

