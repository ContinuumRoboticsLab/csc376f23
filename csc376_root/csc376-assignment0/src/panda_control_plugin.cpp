#include <panda_control_plugin.h>

namespace gazebo
{
	
	/// \brief Constructor
	PandaControlPlugin::PandaControlPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    void PandaControlPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
		
		
		// Check if the plugin is connected to the correct model
		std::cerr << "\nThe Panda plugin is attached to model[" << _model->GetName() << "]\n";
		
		std::cout << "Number of joints is " << _model->GetJointCount() << std::endl;
        
        // Safety check
		if (_model->GetJointCount() == 0)
		{
			std::cerr << "Invalid joint count, iiwa plugin not loaded\n";
			return;
		}

		// Store the model pointer for convenience.
		this->model = _model;
		
		this->world = model->GetWorld();
		
		this->world->Physics()->GetContactManager()->SetNeverDropContacts(true);
		
		// Get the joints.
		this->joints.push_back(_model->GetJoint("panda_joint1"));
		this->joints.push_back(_model->GetJoint("panda_joint2"));
		this->joints.push_back(_model->GetJoint("panda_joint3"));
		this->joints.push_back(_model->GetJoint("panda_joint4"));
		this->joints.push_back(_model->GetJoint("panda_joint5"));
		this->joints.push_back(_model->GetJoint("panda_joint6"));
		this->joints.push_back(_model->GetJoint("panda_joint7"));
		
		_model->GetLink("panda_link0")->SetEnabled(false);
		_model->GetLink("panda_link1")->SetEnabled(false);
		_model->GetLink("panda_link2")->SetEnabled(false);
		_model->GetLink("panda_link3")->SetEnabled(false);
		_model->GetLink("panda_link4")->SetEnabled(false);
		_model->GetLink("panda_link5")->SetEnabled(false);
		_model->GetLink("panda_link6")->SetEnabled(false);
		_model->GetLink("panda_link7")->SetEnabled(false);
		
		// Get the sensors.
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(0)));
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(1)));
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(2)));
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(3)));
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(4)));
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(5)));
		this->sensors.push_back(std::dynamic_pointer_cast<sensors::ContactSensor>(sensors::SensorManager::Instance()->GetSensors().at(6)));

		
		//Get the distal link
		this->ee_link = _model->GetLink("panda_link7");
		
		//Set up node for communication
		this->node = transport::NodePtr(new transport::Node());
		this->node->Init(this->model->GetWorld()->Name());
		

		// Subscribe to the joint topic, and register a callback
		this->sub_joints = this->node->Subscribe("/robot_interface/joint_pos", &PandaControlPlugin::OnJointPosMsg, this);
		mp_contact_subscriber1 = this->node->Subscribe("/gazebo/default/panda/panda_link1/panda_link1_contact/contacts", &PandaControlPlugin::OnContactMsg1, this);
		mp_contact_subscriber2 = this->node->Subscribe("/gazebo/default/panda/panda_link2/panda_link2_contact/contacts", &PandaControlPlugin::OnContactMsg2, this);
		mp_contact_subscriber3 = this->node->Subscribe("/gazebo/default/panda/panda_link3/panda_link3_contact/contacts", &PandaControlPlugin::OnContactMsg3, this);
		mp_contact_subscriber4 = this->node->Subscribe("/gazebo/default/panda/panda_link4/panda_link4_contact/contacts", &PandaControlPlugin::OnContactMsg4, this);
		mp_contact_subscriber5 = this->node->Subscribe("/gazebo/default/panda/panda_link5/panda_link5_contact/contacts", &PandaControlPlugin::OnContactMsg5, this);
		mp_contact_subscriber6 = this->node->Subscribe("/gazebo/default/panda/panda_link6/panda_link6_contact/contacts", &PandaControlPlugin::OnContactMsg6, this);
		mp_contact_subscriber7 = this->node->Subscribe("/gazebo/default/panda/panda_link7/panda_link7_contact/contacts", &PandaControlPlugin::OnContactMsg7, this);
	
		
		// Publish to the pose topic
		this->pub_pose = this->node->Advertise<gazebo::msgs::Pose>("/robot_interface/ee_pose");
		this->pub_collision = this->node->Advertise<gazebo::msgs::Joint>("/robot_interface/collision");
		
		publish_requested = false;
		for(int i = 0; i < 7; i++)
		{
			collision_checked.push_back(false);
			collision_detected.push_back(false);
		}

		
		
		
    }
	
	void PandaControlPlugin::OnJointPosMsg(ConstJointPtr &_msg)
    {
		
		for(int i = 0; i < 7; i++)
		{
			this->joints.at(i)->SetPosition(0,_msg->angle(i),true);
			world->Physics()->UpdatePhysics();
		}
		
		
		
		for(int i = 0; i < 7; i++)
		{
			collision_checked.at(i) = false;
			collision_detected.at(i) = false;
		}
		
		publish_requested = true;
		updateTime = world->SimTime().Double();

    }
    
    void PandaControlPlugin::OnContactMsg1(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)	
			collision_checked.at(0) = true;
		
		if(_msg->contact_size() > 0)
			collision_detected.at(0) = true;
		else
			collision_detected.at(0) = false;
		
		bool all_collisions_checked = true;
		
		int contacts = 0;
		
		for(int i = 0; i < 7; i++)
		{
			if(collision_checked.at(i) == false)
				all_collisions_checked = false;
			
			if(collision_detected.at(i) == true)
				contacts++;
		}
		
	
		if(publish_requested && all_collisions_checked)
		{
			
			
			//Publish updated ee-pose
			gazebo::msgs::Pose msg;
			
			gazebo::msgs::Set(&msg,ee_link->WorldPose());
			
			pub_pose->WaitForConnection();
			pub_pose->Publish(msg);
			
			//Publish collision data
			gazebo::msgs::Joint j_msg;
			j_msg.set_name("Collisions");
			j_msg.add_angle(contacts);	
			pub_collision->WaitForConnection();
			pub_collision->Publish(j_msg);
			
			
			publish_requested = false;
			
			
		}
		
		
	}
	
	void PandaControlPlugin::OnContactMsg2(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)
			collision_checked.at(1) = true;
		
		if(_msg->contact_size() > 0)
			collision_detected.at(1) = true;
		else
			collision_detected.at(1) = false;
			
	}
	
	void PandaControlPlugin::OnContactMsg3(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)
			collision_checked.at(2) = true;
		
		if(_msg->contact_size() > 0)
			collision_detected.at(2) = true;
		else
			collision_detected.at(2) = false;
	}
	
	void PandaControlPlugin::OnContactMsg4(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)
			collision_checked.at(3) = true;
		
		if(_msg->contact_size() > 0)
			collision_detected.at(3) = true;
		else
			collision_detected.at(3) = false;
	}
	
	void PandaControlPlugin::OnContactMsg5(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)
			collision_checked.at(4) = true;
		
		if(_msg->contact_size() > 0)
			collision_detected.at(4) = true;
		else
			collision_detected.at(4) = false;
	}
	
	void PandaControlPlugin::OnContactMsg6(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)
			collision_checked.at(5) = true;
		
		if(_msg->contact_size()> 0)
			collision_detected.at(5) = true;
		else
			collision_detected.at(5) = false;
	}
	
	void PandaControlPlugin::OnContactMsg7(ConstContactsPtr &_msg)
	{
		
		if(publish_requested && gazebo::msgs::Convert(_msg->time()).Double() > updateTime)
			collision_checked.at(6) = true;
		
		if(_msg->contact_size() > 0)
			collision_detected.at(6) = true;
		else
			collision_detected.at(6) = false;
	}


	


    
  
}
