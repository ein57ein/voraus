#ifndef VORAUS_DEMO_H_
#define VORAUS_DEMO_H_

#include "voraus/plug_in_definition.hpp"
#include <eigen3/Eigen/Dense>
#include <std_msgs/Int32.h>
#include <voraus/cpu_demo_ready.h>
#include <voraus/data_demo_ready.h>
#include <stdio.h>

/** 
* regular VORAUS-PlugIns. 
**/
namespace voraus_plugin
{
	class Demo : public voraus::PlugInDefinition
	{
		enum working_state {no, cpu, data};
		working_state working;
		ros::Time startWorkingTime;
		ros::Subscriber agentHeartbeat;
		ros::Subscriber startCpuSub;
		ros::Publisher startCpuPub;
		ros::Subscriber cpuReadySub;
		ros::Subscriber startDataSub;
		ros::Publisher startDataPub;
		ros::Subscriber dataReadySub;
		ros::Publisher dataReturnPub;
		struct agentStruct {
			std::string name;	/**< the name of the agent */
			unsigned int id;	/**< the id of the agent */
			ros::Time lastHeartbeatReceived;	/**< the last time receiving a heartbeat message from this agent */
			bool remove;
			ros::Duration workingDuration;
		};
		std::vector<agentStruct> agent;	/**< a list of all known agents */

		ros::Subscriber cpuDemoSub;
		ros::Publisher cpuReadyPub;
		ros::Subscriber dataDemoSub;
		ros::Publisher dataReadyPub;
		ros::Subscriber dataReturnSub;

		int msgs_counter;
		
	public:
		/** Initialize this class as a task scheduling module
		 */
		Demo() {
			ROS_DEBUG("constructed a VORAUS demo-PlugIn");
			module.module = module_demo;
			module.type = "VORAUS Demo";
		}
		
		/** Intentionally left empty
		 */
		virtual ~Demo() {}

	private:
		/* on central node */
		
		friend bool removeAgent(agentStruct schroedingers_agent);
		
		/** .
		 * @param loopTime time for one Loop of the centralPeriodicCallback
		 * @return error value
		 */
		int onInitCentral(float *loopRate);

		/** .
		 * @param event includes the current and the last time stamps this function was called.
		 */
		void centralPeriodicCallback(const ros::TimerEvent& event);

		void printKnownAgents();
		void agentHeartbeatCallback(const voraus::heartbeat event);
		
		void startCpuDemoCallback(const std_msgs::Int32 event);
		void cpuReadyCallback(const voraus::cpu_demo_ready event);
		
		void startDataDemoCallback(const std_msgs::Int32 event);
		void dataReadyCallback(const voraus::data_demo_ready event);

		/* in nodelet */
		/** .
		 * @param loopTime time for one Loop of the nodeletPeriodicCallback
		 * @return error value
		 */
		int onInitNodelet(float *loopRate);

		/** .
		 * @param event includes the current and the last time stamps this function was called.
		 */
		void nodeletPeriodicCallback(const ros::TimerEvent& event);

		void cpuDemoCallback(const std_msgs::Int32 event);
		void dataDemoCallback(const std_msgs::Int32 event);
		void dataReturnCallback(const voraus::data_demo_ready event);

		/* on decentral node */
		/** .
		 * @param loopTime time for one Loop of the decentralPeriodicCallback
		 * @return error value
		 */
		int onInitDecentral(float *loopRate);
		
		/** .
		 * @param event includes the current and the last time stamps this function was called.
		 */
		void decentralPeriodicCallback(const ros::TimerEvent& event);
	};
}

#endif //VORAUS_DEMO_H_
