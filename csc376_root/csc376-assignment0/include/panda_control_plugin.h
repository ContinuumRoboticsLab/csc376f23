#pragma once

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>


#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <unistd.h>
#include <vector>

namespace gazebo
{
  /// \brief A plugin to control the iiwa in Gazebo.
  class PandaControlPlugin : public ModelPlugin
  {
    
    public:
		
		/// \brief Constructor
		PandaControlPlugin();
	
		/// \brief The load function is called by Gazebo when the plugin is
		/// inserted into simulation
		/// \param[in] _model A pointer to the model that this plugin is
		/// attached to.
		/// \param[in] _sdf A pointer to the plugin's SDF element.
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	
		
     
	private:
		
		bool publish_requested;
		std::vector<bool> collision_checked;
		std::vector<bool> collision_detected;
		double updateTime;

		
		/// \brief Callback for Configuration Message.
		void OnJointPosMsg(ConstJointPtr &_msg);
		
		/// \brief Callback for Collision Message.
		void OnContactMsg1(ConstContactsPtr &_msg);
		void OnContactMsg2(ConstContactsPtr &_msg);
		void OnContactMsg3(ConstContactsPtr &_msg);
		void OnContactMsg4(ConstContactsPtr &_msg);
		void OnContactMsg5(ConstContactsPtr &_msg);
		void OnContactMsg6(ConstContactsPtr &_msg);
		void OnContactMsg7(ConstContactsPtr &_msg);

	
		/// \brief Pointer to the model.
		physics::ModelPtr model;
		physics::WorldPtr world;

	
		/// \brief Pointer to the joints.
		std::vector<physics::JointPtr> joints;
		
		/// \brief Pointer to the sensors.
		std::vector<sensors::ContactSensorPtr> sensors;

		
		/// \brief Pointer to the last link as EE
		physics::LinkPtr ee_link;
		
		/// \brief Nodes used for transport
		transport::NodePtr node;

		/// \brief A subscriber to a named topic.
		transport::SubscriberPtr sub_joints;
		
		/// \brief A subscriber to a named topic.
		transport::SubscriberPtr mp_contact_subscriber1; 
		transport::SubscriberPtr mp_contact_subscriber2; 
		transport::SubscriberPtr mp_contact_subscriber3; 
		transport::SubscriberPtr mp_contact_subscriber4; 
		transport::SubscriberPtr mp_contact_subscriber5; 
		transport::SubscriberPtr mp_contact_subscriber6; 
		transport::SubscriberPtr mp_contact_subscriber7; 

		
		/// \brief A publisher to a named topic.
		transport::PublisherPtr pub_pose;
		transport::PublisherPtr pub_collision;

    
  };
  
  	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(PandaControlPlugin);

  
}
