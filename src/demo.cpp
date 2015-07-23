#include "voraus/demo.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(voraus_plugin::Demo, voraus::PlugInDefinition)

namespace voraus_plugin
{
//---------------------------CENTRAL----------------------------------//
	bool removeAgent(Demo::agentStruct schroedingers_agent){
		return schroedingers_agent.remove;
	}
	
	int Demo::onInitCentral(float *loopRate)
	{
		std::ostringstream temp;
		
		*loopRate = *loopRate / 2;

		agentHeartbeat = node.rosHandle.subscribe("robot_state", 100, &Demo::agentHeartbeatCallback, this);
		
		startCpuSub = node.rosHandle.subscribe("/start_cpu_demo", 100, &Demo::startCpuDemoCallback, this);
		temp.str("");
		temp << name.project_namespace << "start_cpu_demo";	
		startCpuPub = node.rosHandle.advertise<std_msgs::Int32>(temp.str().c_str(), 100);
		temp.str("");
		temp << name.project_namespace << "cpu_demo_ready";
		cpuReadySub = node.rosHandle.subscribe(temp.str().c_str(), 100, &Demo::cpuReadyCallback, this);

		startDataSub = node.rosHandle.subscribe("/start_data_demo", 100, &Demo::startDataDemoCallback, this);
		temp.str("");
		temp << name.project_namespace << "start_data_demo";	
		startDataPub = node.rosHandle.advertise<std_msgs::Int32>(temp.str().c_str(), 100);
		temp.str("");
		temp << name.project_namespace << "data_demo_ready";
		dataReadySub = node.rosHandle.subscribe(temp.str().c_str(), 100, &Demo::dataReadyCallback, this);
		temp.str("");
		temp << name.project_namespace << "data_demo_return";
		dataReturnPub = node.rosHandle.advertise<voraus::data_demo_ready>(temp.str().c_str(), 100);

		working=no;

		msgs_counter=0;

		return 0;
	}

	void Demo::centralPeriodicCallback(const ros::TimerEvent& event) {
		if (plugin_init) {
			ros::Time lastAllowedHeartbeatTime = event.current_real - ros::Duration(2.0);
			for (size_t i=0; i < agent.size(); i++)
			{
				if (agent[i].lastHeartbeatReceived < lastAllowedHeartbeatTime)
				{
					agent[i].remove = true;
				}
			}
			agent.erase(std::remove_if(agent.begin(), agent.end(), removeAgent), agent.end());

			if (agent.size() < 1 && working != no) {
				ROS_WARN("%s all agents are gone. No one will work.", name.full.c_str());
				working = no;
			}
			
			if (working != no) {
				bool allReady = true;
				for ( size_t i = 0; i < agent.size(); i++ )
				{
					if(agent[i].workingDuration <= ros::Duration(0.0)) { allReady = false; }
				}
				
				if (allReady) {
					ros::Duration allOverDuration = ros::Time::now() - startWorkingTime;
					double mean_value = 0, variance = 0;
					for( size_t i = 0; i < agent.size(); i++ ) { mean_value += agent[i].workingDuration.toSec(); }
					mean_value /= agent.size();
					for( size_t i = 0; i < agent.size(); i++ ) { variance += (agent[i].workingDuration.toSec() - mean_value) * (agent[i].workingDuration.toSec() - mean_value);}
					variance /= agent.size();
					double standard_deviation = sqrt(variance);

					std::ostringstream addon;
					addon.str("");

					if (working == data) {
						addon << " - msgs_counter: " << msgs_counter;
					}
					
					ROS_INFO("%s READY in %.3fs (%li agents with a average time of %.3fs and a standard deviation of %.3fs)%s", name.full.c_str(), allOverDuration.toSec(), agent.size(), mean_value, standard_deviation, addon.str().c_str());
					working = no;
				}				
			}
		}
	}

	void Demo::printKnownAgents()
	{
		std::ostringstream temp;
		temp.str("");
		if (agent.size() >= 1) {					
			temp << "found " << agent.size() << " agents: " << agent[0].name << " (id:" << agent[0].id << ")";
			for (int i=1; i < agent.size(); i++) { temp << ", " << agent[i].name << " (id:" << agent[i].id << ")"; }
		} else {
			temp << "found no agents";
		}
		ROS_INFO("%s %s", name.full.c_str(), temp.str().c_str());
	}

	void Demo::agentHeartbeatCallback(const voraus::heartbeat event)
	{
		bool agentInList = false;
		
		for ( size_t i = 0; i < agent.size(); i++ )
		{
			if(agent[i].id == event.id)
			{
				agent[i].lastHeartbeatReceived = event.header.stamp;
				agentInList = true;
			}
		}

		if (!agentInList && working == no) {
			agentStruct temp;
			temp.id = event.id;
			temp.name = event.name;
			temp.lastHeartbeatReceived = event.header.stamp;
			temp.remove = false;
			temp.workingDuration = ros::Duration(0.0);

			agent.push_back(temp);
		}
	}

	void Demo::startCpuDemoCallback(const std_msgs::Int32 event) {
		if (working == no) {
			printKnownAgents();
			for ( size_t i = 0; i < agent.size(); i++ ) { agent[i].workingDuration = ros::Duration(0.0); }
			working = cpu;
			startWorkingTime = ros::Time::now();
			startCpuPub.publish(event);
		}
	}

	void Demo::startDataDemoCallback(const std_msgs::Int32 event) {
		if (working == no) {
			printKnownAgents();
			for ( size_t i = 0; i < agent.size(); i++ ) { agent[i].workingDuration = ros::Duration(0.0); }
			working = data;
			msgs_counter = 0;
			startWorkingTime = ros::Time::now();
			startDataPub.publish(event);
		}
	}

	void Demo::cpuReadyCallback(const voraus::cpu_demo_ready event)
	{
		if (working == cpu) {
			for ( size_t i = 0; i < agent.size(); i++ )
			{
				if(agent[i].id == event.id)	{ agent[i].workingDuration = event.duration; }
			}
		}
	}

	void Demo::dataReadyCallback(const voraus::data_demo_ready event)
	{
		if (working == data) {
			//std::string test = event.data;

			if (event.id == 2) msgs_counter++;
			
			if (event.msgs_left < 1) {
				ros::Time end = ros::Time::now();
				
				for ( size_t i = 0; i < agent.size(); i++ )
				{			
					if(agent[i].id == event.id && agent[i].workingDuration <= ros::Duration(0) )	{
						ros::Duration duration = end - event.start;
				
						//ROS_INFO("%s receive %li of %i chars from agent %i in %.3lfs.",name.full.c_str(), event.data.length(), event.msgs_left, event.id, duration.toSec());
						
						agent[i].workingDuration = duration;
					}
				}
			} else {
				voraus::data_demo_readyPtr temp(new voraus::data_demo_ready);
				*temp = event;
				temp->msgs_left -= 1;
				//ROS_INFO("id_%i, %i", temp->id, temp->msgs_left);
				if (event.id == 2) msgs_counter++;

				dataReturnPub.publish(temp);
			}
		}
	}
		

//---------------------------NODELET----------------------------------//
	int Demo::onInitNodelet(float *loopRate)
	{
		std::ostringstream temp;
		
		temp.str("");
		temp << name.project_namespace << "start_cpu_demo";		
		cpuDemoSub = node.rosHandle.subscribe(temp.str().c_str(), 100, &Demo::cpuDemoCallback, this);
		temp.str("");
		temp << name.project_namespace << "cpu_demo_ready";
		cpuReadyPub = node.rosHandle.advertise<voraus::cpu_demo_ready>(temp.str().c_str(), 100);

		temp.str("");
		temp << name.project_namespace << "start_data_demo";		
		dataDemoSub = node.rosHandle.subscribe(temp.str().c_str(), 100, &Demo::dataDemoCallback, this);
		temp.str("");
		temp << name.project_namespace << "data_demo_ready";
		dataReadyPub = node.rosHandle.advertise<voraus::data_demo_ready>(temp.str().c_str(), 100);
		temp.str("");
		temp << name.project_namespace << "data_demo_return";		
		dataReturnSub = node.rosHandle.subscribe(temp.str().c_str(), 100, &Demo::dataReturnCallback, this);
		
		return 0;
	}

	void Demo::nodeletPeriodicCallback(const ros::TimerEvent& event) {
		if (plugin_init) {
		}
	}

	void Demo::cpuDemoCallback(const std_msgs::Int32 event)
	{
		if (plugin_init) {
			int size = event.data; /* double (float ~double/2): 50 -> ~3ms; 500 -> ~1s; 1000 -> ~7s; 1500 -> ~23s; 2000 -> ~55s*/

			Eigen::MatrixXd m = Eigen::MatrixXd::Random(size, size);
			m = (m + Eigen::MatrixXd::Constant(size, size, 1.0)) * 50;
			Eigen::MatrixXd m2 = Eigen::MatrixXd::Random(size, size);
			m2 = (m2 + Eigen::MatrixXd::Constant(size, size, 1.0)) * 50;
			Eigen::MatrixXd m_;

			//ROS_INFO("%s start with size=%i", name.full.c_str(), size);

			ros::Time start = ros::Time::now();
			m_ = m * m2;
			ros::Time end = ros::Time::now();

			voraus::cpu_demo_ready temp;
			temp.id = name.id;
			temp.duration = end - start;
			cpuReadyPub.publish(temp);

			//ROS_INFO("%s end: %.3lf", name.full.c_str(), (end-start).toSec());
		}
	}

	void Demo::dataDemoCallback(const std_msgs::Int32 event)
	{
		if (plugin_init) {
			
			//generate a string with 200k chars
			std::string text;			
			for (int i = 0; i < 10000; i++) {text += "jlkaoebrqdfsjlkfjcvj";}

			voraus::data_demo_readyPtr temp(new voraus::data_demo_ready);
			temp->id = name.id;
			temp->data = text;
			temp->start = ros::Time::now();

			//for (int i = event.data; i > 0; i = i - 1) {
				temp->msgs_left = event.data - 1;//i - 1;
				dataReadyPub.publish(temp);
			//}
		}
	}

	void Demo::dataReturnCallback(const voraus::data_demo_ready event)
	{
		if (event.id == name.id){
			voraus::data_demo_readyPtr temp(new voraus::data_demo_ready);
			*temp = event;
			temp->msgs_left -= 1;

			dataReadyPub.publish(temp);
		}
	}
	

//--------------------------DECENTRAL---------------------------------//
	int Demo::onInitDecentral(float *loopRate)
	{	
		return 0;
	}

	void Demo::decentralPeriodicCallback(const ros::TimerEvent& event) {
		if (plugin_init) {  }
	}
}
