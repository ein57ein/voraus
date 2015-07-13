/** \file central_mod_man.cpp
 * Create a instance of a central module managemener from the \link 
 * voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file depends on central_module_management.hpp
 **/

#include "module_management/central_module_management.hpp"

/** Create a ROS NodeHandle. Create a Instance of the \link
 * module_management#ModManCentral central module managemener Class
 * \endlink . Excute ros::spin()
 */
int main(int argc, char **argv)
{

	ros::init(argc, argv, "central_modMan");
	ros::NodeHandle roshandle;
	
	module_management::ModManCentral observer(roshandle);
	
	ros::spin();

	return 0;
}
