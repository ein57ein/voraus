/** \file distributed_plug_in.hpp
 * The nodelet part of the plug-in controller in the \link voraus::Voraus 
 * Voraus \endlink framework.
 * Author : Martin Seidel
 *
 * This file includes the \link voraus::DistributedPlugIn 
 * DistributedPlugIn \endlink class. It depends on plug_in_definition.hpp
 **/
#ifndef DISTRIBUTED_PLUG_IN_H_
#define DISTRIBUTED_PLUG_IN_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>

#include "voraus/plug_in_definition.hpp"

namespace voraus
{
	/** This class starts a plugIn. It should be used as nodelet in a 
	 * flexible module node.
	 */
	class DistributedPlugIn : public nodelet::Nodelet, public voraus::Voraus
	{
		pluginlib::ClassLoader<voraus::PlugInDefinition> plug_in_loader;	/**< a pluginLoader **/

		boost::shared_ptr<voraus::PlugInDefinition> plug_in_node;	/**< a pointer to a instances of a plugIn **/

	public:
		/** Intentionally left empty
		 */
		DistributedPlugIn():plug_in_loader("voraus","voraus::PlugInDefinition")
		{
			//NODELET_DEBUG("constructed task_scheduling nodelet");
		}

		/** Intentionally left empty
		 */
		~DistributedPlugIn() {}

		/**  Initialize a instance of Voraus and start a job plugIn.
		 * Will be called after the construcion of the nodelet
		 */
		void onInit();		
	};
}

#endif //DISTRIBUTED_PLUG_IN_H_
