/** \file decentral_mod_man.cpp
 * Create a instance of a decentral module managemener from the \link 
 * voraus::Voraus Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file depends on decentral_module_management.hpp
 **/

#include "module_management/decentral_module_management.hpp"

/** Create a ROS NodeHandle. Create a Instance of the \link
 * module_management#ModManDecentral decentral module managemener Class
 * \endlink . Excute ros::spin().
 */
int main(int argc, char **argv)
{

	ros::init(argc, argv, "decentral_modMan");
	ros::NodeHandle roshandle;

	module_management::ModManDecentral robot(roshandle);
	
	ros::spin();

	return 0;
}
