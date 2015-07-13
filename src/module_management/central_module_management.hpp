/** \file central_module_management.hpp
 * Contains the class definition of the central module managemener in 
 * the \link voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file includes the ModManCentral class. It depends on
 * module_management_base.hpp
 **/
#ifndef CENTRAL_MODULE_MANAGEMENT_H_
#define CENTRAL_MODULE_MANAGEMENT_H_

#include "module_management/module_management_base.hpp"

namespace module_management
{
	/** This class receive state messages from the robots and send a 
	 * list of known robots as Heartbeats. It should be used on a 
	 * central node (server).
	 */
	class ModManCentral : public ModManBase
	{
		ros::ServiceServer robot_registration;	/**< a ros service handler. robots should be register here. */
		ros::Publisher serverHeartbeat;	/**< a publisher for the server heartbeat. */
		ros::Subscriber robotHeartbeat;	/**< a subscriber for heartbeats from the robots / clients */ 

		struct {
			unsigned int nextID;	/**< the next new robot get this id */
			ros::Duration maxTimeWithoutHeartbeat;	/**< after this time a robot will be marked as lost */
		}server;	/**< relevant informations about this server node */

		/** informations about a registered robot */
		struct robotStruct {
			std::string name;	/**< the name of the robot */
			unsigned int id;	/**< the id of the robot */
			ros::Time lastHeartbeatSend;	/**< the last time the robot (his id) was named in the server-heartbeat-message **/
			ros::Time lastHeartbeatReceived;	/**< the last time a heartbeat message with the robot id was received */ 
			robot_states state;	/**< the state of the robot send with the last heartbeat */
		};

		std::vector<robotStruct> robots;	/**< a list of all known/registered robots */

	public:
		/** Initialize the #server structure, the registration service
		 * and the heartbeat related publisher and subscriber
		 * @param roshandle a nodehandle from roscore.
		**/
		ModManCentral(ros::NodeHandle roshandle);
		
	private:
		/** Append a new robot to the #robots list. The response
		 * contains the id for the robot and the module configuration.
		 * @param req the informations from the robot
		 * @param res the informations for the robot
		 * @return true if the registration was successfully
		 */
		bool robotLogIn(voraus::robot_registration::Request &req, voraus::robot_registration::Response &res);

		/** Reads the module configuration from the parameter server.
		 * @return error code
		 */
		int getModuleConfiguration();

		/** Send the server heartbeat. A Heartbeat contain the number of
		 * known robots and the information about one of this robots
		 * @return error code
		 */
		int sendHeartbeat();

		/** Checks if the robots send their last heartbeat within the
		 * \link #maxTimeWithoutHeartbeat right time \endlink and call
		 * sendHeartbeat().
		 * @param event contain different times related to the calling timer
		 * @return error code
		 */
		int loop(const ros::TimerEvent& event);

		/** Receives the heartbeat messages from the robots. Print a
		 * warning if a unregistered robot send a heartbeat message.
		 * @param event information about the sending robot
		 */
		void robotHeartbeatCallback(const voraus::heartbeat event);

		/** Removes a robot from the ModManCentral#robots list if his state is \"Voraus#lost\"
		 * @param dead_robot information about the robot, which should be checked
		 * @return if true the robot will be removed
		 */
		friend bool removeRobot(robotStruct dead_robot);
	};
};

#endif /*CENTRAL_MODULE_MANAGEMENT_H_*/

