/** \file central_module_management.cpp
 * Contains the class functions of the central module managemener in 
 * the \link voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file depends on central_module_management.hpp
 **/
 
#include "central_module_management.hpp"

namespace module_management
{
	bool removeRobot(ModManCentral::robotStruct dead_robot){
		return (dead_robot.state == voraus::Voraus::lost) ? true : false;
	}

	ModManCentral::ModManCentral(ros::NodeHandle roshandle):ModManBase(roshandle, true)
	{
		name.id = 1;
		server.nextID = 2;
		
		buildFullName();

		float max_time_wo_heartbeat;
		ros::param::param<float>(ros::names::append(name.project_namespace, "max_time_without_heartbeat").c_str(), max_time_wo_heartbeat, 1.0);
		max_time_wo_heartbeat = (max_time_wo_heartbeat < 0.1 || max_time_wo_heartbeat > 300.0) ? 1.0 : max_time_wo_heartbeat;
		server.maxTimeWithoutHeartbeat = ros::Duration(max_time_wo_heartbeat);

		//ROS_INFO("%s timeOut: %.2fs", name.full.c_str(), server.maxTimeWithoutHeartbeat.toSec());

		getModuleConfiguration();

		robot_registration = node.rosHandle.advertiseService("robot_registration", &ModManCentral::robotLogIn, this);
		serverHeartbeat = node.rosHandle.advertise<voraus::heartbeat>("known_robot_list", 50);
		robotHeartbeat = node.rosHandle.subscribe("robot_state", 50, &ModManCentral::robotHeartbeatCallback, this);	
	}

	bool ModManCentral::robotLogIn(voraus::robot_registration::Request &req, voraus::robot_registration::Response &res)
	{
		//Warning if roboter-name is already in use
		int name_in_use_with_id = 0;
		for( size_t i = 0; i < robots.size(); i++ )
		{
			if ( robots[i].name == req.name ) {	name_in_use_with_id = robots[i].id; }
		}
		
		if (name_in_use_with_id > 1) {
			ROS_WARN("%s robot name \"%s\" is allready in use. Response again with id=%i.", name.full.c_str(), req.name.c_str(), name_in_use_with_id);	// This possible cause problems with the informations in the parameter server.
			res.id = name_in_use_with_id;
		} else {
			robotStruct tempRobot;
			tempRobot.name = req.name;
			tempRobot.lastHeartbeatReceived = ros::Time::now();
			tempRobot.lastHeartbeatSend = tempRobot.lastHeartbeatReceived;
			tempRobot.state = idle;
			tempRobot.id = server.nextID;
			server.nextID++;
			
			robots.push_back(tempRobot);
			
			ROS_INFO("%s register robot \"%s\" with id=%i", name.full.c_str(), tempRobot.name.c_str(), tempRobot.id);
			
			res.id = tempRobot.id;
		}
				
		res.timeout = server.maxTimeWithoutHeartbeat.toSec();
		res.task_scheduling = central.task_scheduling;
		res.localisation = central.localisation;
		res.path_planing = central.path_planing;
		
		return true;
	}

	int ModManCentral::getModuleConfiguration()
	{
		std::ostringstream temp;
		
		temp.str("");
		temp << name.project_namespace << "/module_config/task_scheduling_central";
		ros::param::param<bool>(temp.str().c_str(), central.task_scheduling, true);

		temp.str("");
		temp << name.project_namespace << "/module_config/localisation_central";
		ros::param::param<bool>(temp.str().c_str(), central.localisation, false);
		
		temp.str("");
		temp << name.project_namespace << "/module_config/path_planing_central";
		ros::param::param<bool>(temp.str().c_str(), central.path_planing, false);

		temp.str("");
		temp << "Module-Config: ";
		temp << "task_scheduling = " << ((central.task_scheduling) ? "central" : "decentral");
		temp << ", localisation = " << ((central.localisation) ? "central" : "decentral");
		temp << ", path_planing = " << ((central.path_planing) ? "central" : "decentral");

		ROS_INFO("%s TimeOut: %.2fs; %s", name.full.c_str(), server.maxTimeWithoutHeartbeat.toSec(), temp.str().c_str());
		
		return 0;
	}

	int ModManCentral::loop(const ros::TimerEvent& event)
	{
		//a robot will be removed if he didn't send a heartbeat-message after this time
		ros::Time lastAllowedHeartbeatTime = event.current_real - server.maxTimeWithoutHeartbeat;
		for (size_t i=0; i < robots.size(); i++)
		{
			if (robots[i].lastHeartbeatReceived < lastAllowedHeartbeatTime)
			{
				robots[i].state = lost;
				ROS_ERROR("%s [%i %s] robot is lost. Removing from list", name.full.c_str(), robots[i].id, robots[i].name.c_str());
			}
		}
		robots.erase(std::remove_if(robots.begin(), robots.end(), removeRobot), robots.end());
		
		//a warning will be printed if a robot didn't send a heartbeat-message after this time
		ros::Time criticalHeartbeatTime = event.current_real - server.maxTimeWithoutHeartbeat * 0.7;
		for( size_t i = 0; i < robots.size(); i++ )
		{
			if (robots[i].lastHeartbeatReceived < criticalHeartbeatTime) {ROS_WARN("%s [%i %s] no heartbeat since %.2f sec", name.full.c_str(), robots[i].id, robots[i].name.c_str(), ros::Time::now().toSec() - robots[i].lastHeartbeatReceived.toSec());}
		}

		sendHeartbeat();

		return 0;
	}

	int ModManCentral::sendHeartbeat()
	{
		int robot_count = robots.size();

		voraus::heartbeat temp_heartbeat;
		temp_heartbeat.header.stamp = ros::Time::now();
		
		if (robot_count > 0)
		{
			int current_robot_id = 0;
			
			temp_heartbeat.current_known_robot_count = robot_count;
			
			for ( size_t i = 1; i < robot_count; i++ )
			{
				if (robots[current_robot_id].lastHeartbeatSend > robots[i].lastHeartbeatSend) {current_robot_id = i;}
			}

			temp_heartbeat.name = robots[current_robot_id].name;
			temp_heartbeat.id = robots[current_robot_id].id;
			temp_heartbeat.robotState = robots[current_robot_id].state;

			robots[current_robot_id].lastHeartbeatSend = temp_heartbeat.header.stamp;
		} else {
			temp_heartbeat.current_known_robot_count = 0;
			temp_heartbeat.name = "";
			temp_heartbeat.id = 1;
			temp_heartbeat.robotState = 1;
		}

		serverHeartbeat.publish(temp_heartbeat);

		return 0;
	}

	void ModManCentral::robotHeartbeatCallback(const voraus::heartbeat event)
	{
		bool robotInList = false;
		
		for ( size_t i = 0; i < robots.size(); i++ )
		{
			if(robots[i].id == event.id)
			{
				robots[i].lastHeartbeatReceived = event.header.stamp;
				robots[i].state = (robot_states) event.robotState;
				robotInList = true;
			}
		}

		if (!robotInList) {ROS_WARN("%s received a message from a robot (id:%i, state:%i) which is not registered.", name.full.c_str(), event.id, event.robotState);}
	}
}
