/** \file decentral_module_management.cpp
 * Contains the class functions of the decentral module managemener in 
 * the \link voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file depends on decentral_module_management.hpp
 **/

#include "decentral_module_management.hpp"

namespace module_management
{
	ModManDecentral::ModManDecentral(ros::NodeHandle roshandle):ModManBase(roshandle, false)
	{
		std::ostringstream temp;
		temp.str("");
		temp << name.project_namespace << "robot_registration";
		logIn = node.rosHandle.serviceClient<voraus::robot_registration>(temp.str().c_str());

		robot.state = voraus::Voraus::unregistered;

		robot.lastHeartbeatWithMyIdCounter = 0;
		temp.str("");
		temp << name.project_namespace << "known_robot_list";
		serverHeartbeat = node.rosHandle.subscribe(temp.str().c_str(), 50, &ModManDecentral::serverHeartbeatCallback, this);

		temp.str("");
		temp << name.project_namespace << "robot_state";
		robotHeartbeat = node.rosHandle.advertise<voraus::heartbeat>(temp.str().c_str(), 50);
	}

	int ModManDecentral::loop(const ros::TimerEvent& event)
	{
		if (robot.state == voraus::Voraus::idle || robot.state == voraus::Voraus::working || robot.state == voraus::Voraus::failure)
		{
			//the robot will be unregistered if he didn't send a heartbeat-message after this time
			if (robot.lastHeartbeatTime < event.current_real - robot.maxTimeWithoutHeartbeat)
			{
				ROS_ERROR("%s master is lost. Trying to reconnect.", name.full.c_str());
				registerAtServer();
			}
			
			//a warning will be printed if the master didn't send a heartbeat-message after this time
			if (robot.lastHeartbeatTime < event.current_real - robot.maxTimeWithoutHeartbeat * 0.7)
			{
				ROS_WARN("%s no heartbeat from master since %.2f sec", name.full.c_str(), event.current_real.toSec() - robot.lastHeartbeatTime.toSec());
			}
			
			sendHeartbeat();
		}
	}

	int ModManDecentral::sendHeartbeat()
	{
		voraus::heartbeat temp_heartbeat;
		
		temp_heartbeat.header.stamp = ros::Time::now();
		temp_heartbeat.current_known_robot_count = 1;
		temp_heartbeat.id = name.id;
		temp_heartbeat.name = name.name;
		temp_heartbeat.robotState = robot.state;

		robotHeartbeat.publish(temp_heartbeat);

		return 0;
	}

	int ModManDecentral::registerAtServer()
	{
		voraus::robot_registration logInTemp;
		logInTemp.request.name = name.name;

		if (logIn.call(logInTemp))
		{
			robot.state = voraus::Voraus::idle;
			name.id = logInTemp.response.id;
			robot.maxTimeWithoutHeartbeat = ros::Duration(logInTemp.response.timeout);
			central.task_scheduling = logInTemp.response.task_scheduling;
			central.localisation = logInTemp.response.localisation;
			central.path_planing = logInTemp.response.path_planing;
			central.demo = logInTemp.response.demo;
		} else {
			robot.state = voraus::Voraus::unregistered;
			name.id = 0;
			robot.maxTimeWithoutHeartbeat = ros::Duration(3600);
			central.task_scheduling = true;
			central.localisation = true;
			central.path_planing = true;
			central.demo = true;
		}

		robot.lastHeartbeatWithMyIdCounter = 0;

		buildFullName();

		if (name.id == 0)
		{
			ROS_ERROR("%s server unreachable", name.full.c_str());
		} else {
			std::ostringstream temp;
			temp.str("");
			temp << "Module-Config: ";
			temp << "task_scheduling = " << ((central.task_scheduling) ? "central" : "decentral");
			temp << "; localisation = " << ((central.localisation) ? "central" : "decentral");
			temp << "; path_planing = " << ((central.path_planing) ? "central" : "decentral");
			temp << "; demo = " << ((central.demo) ? "central" : "decentral");

			ROS_INFO("%s Connected to server; TimeOut: %.2fs; %s", name.full.c_str(), robot.maxTimeWithoutHeartbeat.toSec(), temp.str().c_str());
		}

		return 0;
	}

	void ModManDecentral::serverHeartbeatCallback(const voraus::heartbeat event)
	{
		if (event.id == name.id)
		{
			robot.lastHeartbeatWithMyIdCounter = 0;
			if (event.robotState != robot.state) { ROS_WARN("%s server has a wrong state for this robot (%i instead of %i)", name.full.c_str(), event.robotState, robot.state);}
		} else {
			robot.lastHeartbeatWithMyIdCounter++;
		}

		if (robot.lastHeartbeatWithMyIdCounter >= event.current_known_robot_count * 2)
		{
			registerAtServer();
		}

		robot.lastHeartbeatTime = event.header.stamp;
	}

}
