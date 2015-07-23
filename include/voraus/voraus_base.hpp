/** \file voraus_base.hpp
 * Basic functions of the \link voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file includes the \link voraus::Voraus Voraus \endlink class and
 * a attendant \link voraus::VorausException Exception \endlink class. 
 * It depends on module_registration.srv (module_registration.h) and 
 * heartbeat.msg (heartbeat.h).
 **/
/** \file module_registration.srv
 * Config file for a ros service to register a module at the local 
 * module manager node.
 * Author : Martin Seidel
 * 
 * Catkin use this file to generate the module_registration.h.
 **/
 /** \file heartbeat.msg
 * Config file for a ros message, which will be used to send heartbeats 
 * with information about the known robots.
 * Author : Martin Seidel
 * 
 * Catkin use this file to generate the heartbeat.h.
 **/
#ifndef VORAUS_BASE_H_
#define VORAUS_BASE_H_

#include "ros/ros.h"

#include "voraus/heartbeat.h"
#include "voraus/module_registration.h"

/** Makro - short way throwing an Exception */
#define THROW(EC) throw voraus::VorausException(__FILE__, __LINE__, EC);

/** Basic functions of the VORAUS framework.
 */
namespace voraus
{
	/** Base class of the Voraus Framework. This class provides 
	 * functions to read and set the name parameter of a node and a 
	 * function which register a module at the module manager. It also 
	 * start a ros::Timer as main loop with a previous read parameter.
	 */
	class Voraus
	{
		/** Information about the current mode of the node */
		struct node_parameter	
		{
			bool onServer;	/**< indicates if this node runs on a server. */
			ros::NodeHandle rosHandle;	/**< a handle for starting Timer, Subscriber, Publisher, ServiceClients and ServiceServer */
		};

	public:
		/** All possible states of the robot **/
		enum robot_states	
		{
			unregistered = voraus::heartbeat::robot_unregistered,	/**< robot has no contact to the server-node **/
			idle = voraus::heartbeat::robot_idle,	/**< robot has contact to server-node and is ready for work **/
			working = voraus::heartbeat::robot_working,	/**< robot has contact to server-node and is doing something **/
			failure = voraus::heartbeat::robot_failure,	/**< robot has contact to server-node and has detected a failure **/
			lost = voraus::heartbeat::robot_lost	/**< robot has no contact to server-node; only used by the server if the robot heartbeat wasn't received for a longer time **/
		};

		/** All definded functions, which a module could have */
		enum known_module_types	
		{
			module_undefind = voraus::module_registration::Request::undefind,	/**< default value; couldn't be registered */
			module_modMan = voraus::module_registration::Request::module_managment,	/**< a module, which manage other modules */
			module_taSche = voraus::module_registration::Request::task_scheduling,	/**< a module, which schedules tasks */
			module_local = voraus::module_registration::Request::localisation,	/**< a module, which locate the robot in a (un)known map */
			module_paPlan = voraus::module_registration::Request::path_planing,	/**< a module, which generates a trajectorie from the current and a target position */
			module_movBase = voraus::module_registration::Request::move_base,	/**< a module, which is used to move the robot */
			module_maniCon = voraus::module_registration::Request::manipulator_controller,	/**< a module, which is used to controll a manipulator. */
			module_fluidCon = voraus::module_registration::Request::fluid_spout_controller,	/**< a module, which is used inside the robomix package. It communicate with the arduino controller. */ //maybe misc?
			module_misc = voraus::module_registration::Request::misc,		/**< a module, which does not match one of the other functions */
			module_demo = voraus::module_registration::Request::demo,
		};

	protected:
		/** Provides all values making a node unique**/
		struct name_parameter
		{
			std::string full;	/**< name of the node including the namespace of the current project, module name and id**/
			std::string name; /**< name of the node **/
			std::string project_namespace;	/**< namespace of the current project **/
			std::string module;	/**< name of this module */
			unsigned int id;	/**< unique id for each robot. 1 = server, 0 = unregistered respectively stand-a-alone **/
		};

		ros::ServiceClient moduleLogIn;	/**< client, which is used to get the module_config from the corresponding module management node **/
		ros::Timer mainLoopTimer;	/**< timer for the mainLoopCallback **/
		name_parameter name;	/**< all name parameter for this node */
		node_parameter node;	/**< a nodeHandle and the information if this node runs on a server */

		struct {
			known_module_types module;	/**< what does this module */
			std::string type;	/**< Describtion of this module */
			bool central;	/**< indicates if this module runs in central or decentral mode */
		} module;	/**< contains module specific information */

		/** Set default values for #module
		 */
		Voraus() {
			module.module = module_undefind;
			module.type = "";
			module.central = false;
		}

	public:
		/** Intentionally left empty
		 */
		virtual ~Voraus() {}

		/** Return the \link #known_module_types type \endlink of this 
		 * module.
		 * @return type of this module
		 */
		known_module_types get_module() { return module.module; }

		/** Return the description of this module.
		 * @return description of this module.
		 */
		std::string get_type() { return module.type; }
		
	private:
		/** Dig the name of the node and the namespace of the project
		 * from the Node-Namespace.
		 * 	@return error code
		 **/
		int getNameStrings();		

		/** This function test if this node is registered and provides a
		 *  main loop for all inheriting classes.
		 * @param event includes the current and the last time stamps this function was called by roscore
		 */
		void mainLoopCallback(const ros::TimerEvent& event) {
			if (name.id == 0 && module.type != "") {
				registerThisModule();
			}
			vorausLoop(event);
		}

	protected:
		/** Initialize the node. Set the #node and #name variables.
		 * Checks if a valid name and namespace was set. Otherwise it
		 * calls getNameStrings(). Also a loop time will be read from
		 * the parameter server and used to start the mainLoopCallback()
		 * @param roshandle a valid ros::NodeHandle
		 * @param onServer indicates if the node runs on a server
		 * @param module this string will be the tail of the \link #name full string of name \endlink . It should give a idenfication, which module is printing the message.
		 * @param name the name of the node. should be set in plugIns
		 * @param project_namespace the namespace of the project. should be set in plugIns
		 * @param id the id of the node. should be set in plugIns
		 * @param noMainLoopTimer if it's true the Timer for the mainLoopCallback() will not be started
		 * @return the loop rate read from the parameter server
		 */
		float initializeVoraus(ros::NodeHandle roshandle, bool onServer, std::string module, std::string name, std::string project_namespace, unsigned int id, bool noMainLoopTimer);

		/** Calls \link initializeVoraus(ros::NodeHandle,bool,std::string,std::string,std::string,unsigned int,bool) 
		 * initializeVoraus() \endlink with minimal settings.
		 */
		float initializeVoraus(ros::NodeHandle roshandle, bool onServer, std::string module) {
			return initializeVoraus(roshandle, onServer, module, "", "", 0, false);
		}

		/** Calls \link initializeVoraus(ros::NodeHandle,bool,std::string,std::string,std::string,unsigned int,bool) 
		 * initializeVoraus() \endlink with minimal settings for #name.
		 */
		float initializeVoraus(ros::NodeHandle roshandle, bool onServer, std::string module, bool noMainLoopTimer) {
			return initializeVoraus(roshandle, onServer, module, "", "", 0, noMainLoopTimer);
		}

		/** Calls \link initializeVoraus(ros::NodeHandle,bool,std::string,std::string,std::string,unsigned int,bool) 
		 * initializeVoraus() \endlink using \link Voraus::name_parameter name_parameter \endlink 
		 * and with a mainLoopTimer.
		 */
		float initializeVoraus(ros::NodeHandle roshandle, bool onServer, name_parameter name) {
			return initializeVoraus(roshandle, onServer, name.module, name.name, name.project_namespace, name.id, false);
		}

		/** Calls \link initializeVoraus(ros::NodeHandle,bool,std::string,std::string,std::string,unsigned int,bool) 
		 * initializeVoraus() \endlink using \link Voraus::name_parameter name_parameter \endlink .
		 */
		float initializeVoraus(ros::NodeHandle roshandle, bool onServer, name_parameter name, bool noMainLoopTimer) {
			return initializeVoraus(roshandle, onServer, name.module, name.name, name.project_namespace, name.id, noMainLoopTimer);
		}
		
		/** Build \link Voraus::name_parameter#full name.full \endlink 
		 * from all others values of #name
		 * @param sufix the tail of \link Voraus::name_parameter#full full \endlink
		 * @param prefix leading string of \link Voraus::name_parameter#full full \endlink
		 * @return error code
		 */
		int buildFullName(std::string sufix = "]", std::string prefix = "[");

		/** Will be called from the mainLoopCallback(). Could be used by
		 * inheritied classes as loop.
		 * @param event includes the current and the last time stamps mainLoopCallback() was called by roscore
		 * @return error code
		 */
		virtual int vorausLoop(const ros::TimerEvent& event) {return 0;}

		/** Calls the module-regastration-service of the local modMan-node.
		 * 	@return error code
		 **/
		int registerThisModule();
	};

	/** The Exception class for the Voraus framework
	 */
	class VorausException : public std::exception
	{
		std::string errorText; /**< contains the current error text. will be generated in VorausException() and published in what() */
		
	public:
		/**< All possible errors */
		enum errors
		{
			no_name_rob,	/**< a decentral node was started in ros root namespace */
			no_plug_in,	/**< a invalid plug_in name was read from the parameter server */
			test_error	/**< just testing if a exception is throwen, when called */
		};

		/** Build a fine text line from the given information.
		 * @param file name of calling file
		 * @param line on which line is the error?
		 * @param error which error is it?
		 */
		VorausException(const char* file, unsigned int line, errors error) throw()
		{
			std::ostringstream temp;
			temp.str("");
			temp << "[" << file << " | line " << line << "] ";
			
			if (error == no_name_rob) {
				temp << "Decentral node was started without a name (= empty namespace). This name will be generated from the namespace of the node and will be used to get informations from the parameter server.";
			} else if (error == no_plug_in) {
				temp << "No plugIn was given. It's impossible to load a plugIn.";
			} else if (error == test_error) {
				temp << "Test ERROR";
			}

			this->errorText = temp.str();
		}

		/** Intentionally left empty.
		 * What else did you expected?
		 */
		~VorausException() throw() {}

		/** Published the #errorText.
		 * @return the #errorText ...
		 */
		const char* what() const throw() { return errorText.c_str(); }		
	};

	int Voraus::getNameStrings()
	{
		std::string temp_name = ros::names::clean( ros::this_node::getNamespace() );

		if (node.onServer)
		{
			name.name = "master";
			name.project_namespace = temp_name;
		} else {
			size_t x = temp_name.find_last_of("/");
			name.project_namespace = temp_name.substr(0, x+1);
			name.name = temp_name.substr(x+1, temp_name.length()-x-1);
			if (name.name.length() < 1) {THROW(voraus::VorausException::no_name_rob);}
		}

		if (name.project_namespace.length() < 3)
		{
			name.project_namespace = "/";
		} else {
			if (node.onServer) {name.project_namespace.append("/");}
		}

		return 0;
	}

	float Voraus::initializeVoraus(ros::NodeHandle roshandle, bool onServer, std::string module, std::string name, std::string project_namespace, unsigned int id, bool noMainLoopTimer)
	{
		node.rosHandle = roshandle;
		node.onServer = onServer;
		this->name.module = module;
		this->name.id = id;

		if (name == "" || project_namespace == "")
		{
			getNameStrings();
		} else {
			this->name.name = name;
			this->name.project_namespace = project_namespace;
		}

		buildFullName();

		moduleLogIn = node.rosHandle.serviceClient<voraus::module_registration>("module_registration");

		float loopRate;
		ros::param::param<float>("~loop_rate", loopRate, 20.0);
		loopRate = (loopRate < 1.0 || loopRate > 5000.0) ? 20.0 : loopRate;

		if (!noMainLoopTimer)
		{
			ROS_INFO("%s loopRate: %.2fHz", this->name.full.c_str(), loopRate);

			mainLoopTimer = node.rosHandle.createTimer(ros::Rate(loopRate), &Voraus::mainLoopCallback, this);
		}
		
		return loopRate;
	}

	int Voraus::buildFullName(std::string sufix, std::string prefix)
	{
		int first = 0, count = name.project_namespace.length(); //indicates leading and trailing "/" in project_namespace
		if (count > 2)
		{
			if (name.project_namespace.compare(count-1,1,"/") == 0) {count = count - 1;}
			if (name.project_namespace.compare(0,1,"/") == 0)
			{
				first = 1;
				count = count - 1;
			}				
		}
		
		std::ostringstream temp;
		temp.str("");
		temp << prefix << name.project_namespace.substr(first, count) << " " << name.id << " " << name.name << " " << name.module << sufix;
		name.full = temp.str();

		return 0;
	}

	int Voraus::registerThisModule()
	{
		if (module.module == module_modMan) { return 1; }
		
		voraus::module_registration logInTemp;
		logInTemp.request.sending_module = module.module;
		logInTemp.request.type = module.type;

		if (moduleLogIn.call(logInTemp))
		{
			name.id = logInTemp.response.id;
			module.central = logInTemp.response.module_central;

			buildFullName();
			if (name.id != 0) {
			ROS_INFO("%s connected to modMan in %s mode", name.full.c_str(), (module.central) ? "central" : "decentral");
			}
		} else {
			name.id = 0;
			module.central = false;

			buildFullName();
			ROS_WARN("%s modMan unreachable", name.full.c_str());
		}
		return 0;
	}
};

#endif /*VORAUS_BASE_H_*/
