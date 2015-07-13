/** \file central_plug_in.cpp
 * Contains functions of the \link voraus::PlugInCentral PlugInCentral
 * \endlink class.
 * Author : Martin Seidel
 *
 * This file depends on central_plug_in.hpp
 **/
#include "central_plug_in.hpp"

namespace voraus
{
	bool removeRobot(PlugInCentral::robot_list_params dead_robot){	
		return (dead_robot.state == voraus::Voraus::lost) ? true : false;
	}

	PlugInCentral::PlugInCentral(ros::NodeHandle roshandle, std::string module_log_name, std::string plug_in_name):PlugInBase(roshandle, true, module_log_name, plug_in_name)
	{
		std::ostringstream temp;
		temp.str("");
		temp << name.project_namespace << "known_robot_list";
		serverHeartbeat = roshandle.subscribe(temp.str().c_str(), 50, &PlugInCentral::serverRobotListCallback, this);
	}

	int PlugInCentral::plugInLoop(const ros::TimerEvent& event)
	{
		for ( size_t i = 0; i < robot_list.size(); i++ )
		{
			if(robot_list[i].state == voraus::Voraus::lost)
			{
				ROS_WARN("%s [%i %s] removing robot from list", name.full.c_str(), robot_list[i].id, robot_list[i].name.c_str());
			}
		}
		
		robot_list.erase(std::remove_if(robot_list.begin(), robot_list.end(), removeRobot), robot_list.end());

		return 0;
	}

	void PlugInCentral::serverRobotListCallback(const voraus::heartbeat event)
	{
		if (event.id > 1 && name.id > 0)
		{
			bool known_robot = false;
			
			for ( size_t i = 0; i < robot_list.size(); i++ )
			{
				if (event.id == robot_list[i].id)
				{
					known_robot=true;
					robot_list[i].state = (robot_states) event.robotState;
					robot_list[i].lastHeartbeatWithRobotIdCounter=0;
				} else {
					robot_list[i].lastHeartbeatWithRobotIdCounter++;
				}
			}

			if (!known_robot)
			{
				robot_list_params new_robot;
				new_robot.id = event.id;
				new_robot.name = event.name;
				new_robot.state = (robot_states) event.robotState;
				new_robot.lastHeartbeatWithRobotIdCounter=0;
				ROS_INFO("%s found new robot on server list [%i %s]", name.full.c_str(), event.id, event.name.c_str());
				if (module.central) { createNodelet(&(new_robot.node), new_robot.id, new_robot.name); }
				robot_list.push_back(new_robot);
			}
		}

		int removing = 0;
		for ( size_t i = 0; i < robot_list.size(); i++ )
		{
			if (robot_list[i].lastHeartbeatWithRobotIdCounter >= event.current_known_robot_count + 1 || event.current_known_robot_count == 0)
			{
				robot_list[i].state = lost;
				removing++;
			}
		}

		if (robot_list.size()-removing != event.current_known_robot_count && name.id > 0)
		{
			ROS_WARN("%s robot lists size do not fit (taSche: %li; modMan: %i)", name.full.c_str(), robot_list.size(), event.current_known_robot_count);
		}
	}
}

