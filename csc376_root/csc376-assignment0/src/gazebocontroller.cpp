#include <gazebocontroller.h>


	
/// \brief Constructor
GazeboController::GazeboController()
{
	// Load gazebo as a client
	gazebo::client::setup();
	
	
	//Ellipsoid visualization
	matMsgSphere = markerMsgSphere.mutable_material();
	matMsgSphere->mutable_script()->add_uri("custom.material");
	matMsgSphere->mutable_script()->set_name("SphereMaterial");
	
	
	// EE-Frame Visualization
	
	//X-Axis
	matMsgX = markerMsgX.mutable_material();
	matMsgX->mutable_script()->set_name("Gazebo/Red");
	
	
	//Y-Axis
	matMsgY = markerMsgY.mutable_material();
	matMsgY->mutable_script()->set_name("Gazebo/Green");
	
	
	//Z-Axis
	matMsgZ = markerMsgZ.mutable_material();
	matMsgZ->mutable_script()->set_name("Gazebo/Blue");
	
	// Create our node for communication
	mp_gazebo_node = gazebo::transport::NodePtr(new gazebo::transport::Node());
	mp_gazebo_node->Init("default");
	
	// Subscribe to the joint topic, and register a callback
	mp_ee_pose_subscriber = mp_gazebo_node->Subscribe("/robot_interface/ee_pose", &GazeboController::OnEEPoseMsg, this);
	mp_collision_subscriber = mp_gazebo_node->Subscribe("/robot_interface/collision", &GazeboController::OnCollisionMsg, this);


	// Publish to the configuration topic
	mp_joint_pos_publisher = mp_gazebo_node->Advertise<gazebo::msgs::Joint>("/robot_interface/joint_pos");
	
	
	
	//Set initial member variables
	m_robot_config.resize(7,1);
	m_robot_config.setZero();
	
	//Initialize zero configuration
	m_ee_frame_valid = false;
	m_collision = false;
	m_collision_valid = false;

	setRobotConfiguration(m_robot_config);
	
	while(!isFrameValid()) { }
	
}

GazeboController::~GazeboController()
{	
	// Close gazebo client
    gazebo::client::shutdown();
	
}

void GazeboController::setRobotConfiguration(Eigen::MatrixXd q)
{
	if(q.rows() != 7 || q.cols() != 1)
	{
		std::cout << "Wrong dimensions of robot configuration!" << std::endl;
		return;
	}
	
	//Get rid of redundancy in angle representation
	for(int i = 0; i < 7; i++)
	{
		while(q(i) > M_PI || q(i) < -M_PI)
		{
			if(q(i) < 0)
				q(i) += 2*M_PI;
			else
				q(i) -= 2*M_PI;
		}
		
	}
	
	m_ee_frame_valid = false;
	m_collision_valid = false;
	
	m_robot_config = q;
	
	// Create a joint message
	gazebo::msgs::Joint msg;
	msg.set_name("Joints");
	
	for(int i = 0; i < q.rows(); i++)
	{
		msg.add_angle(q(i));	
	}
	
	//Publish the message
	mp_joint_pos_publisher->WaitForConnection();
	mp_joint_pos_publisher->Publish(msg);
}

void GazeboController::OnEEPoseMsg(ConstPosePtr &_msg)
{
	ignition::math::Pose3d pose = gazebo::msgs::ConvertIgn(*_msg);
	ignition::math::Matrix4<double> frame(pose);
	
	for(int i = 0; i < 4; i++)
	{
		for(int j = 0; j < 4; j++)
		{
			m_robot_ee_frame(i,j) = frame(i,j);
		}
	}
	
	
	//Pose of link 7 is defined in its origin, we need to translate along it's z-axis by an offset (0.1070 meter from data sheet)
	
	m_robot_ee_frame.block(0,3,3,1) = m_robot_ee_frame.block(0,3,3,1) + m_robot_ee_frame.block(0,0,3,3)*Eigen::Vector3d(0,0,0.1070);
	
	m_ee_frame_valid = true;
}

void GazeboController::OnCollisionMsg(ConstJointPtr &_msg)
{
	int contacts = _msg->angle(0);
	
	if(contacts > 0)
		m_collision = true;
	else
		m_collision = false;
		
	m_collision_valid = true;
}

bool GazeboController::getCollisionState()
{	
	return m_collision;
}
		
		
Eigen::Matrix4d GazeboController::getCurrentRobotEEFrame()
{
	return m_robot_ee_frame;
}
		
Eigen::MatrixXd GazeboController::getCurrentRobotConfiguration()
{
	return m_robot_config;
}

void GazeboController::moveEEFrameVis(Eigen::Matrix4d ee_pose)
{
	
	double length = 0.1;
	double radius = 0.01;
	
	Eigen::Vector3d x_pos_offset(length/2,0,0);
	Eigen::Vector3d y_pos_offset(0,length/2,0);
	Eigen::Vector3d z_pos_offset(0,0,length/2);
	
	Eigen::Matrix3d x_rot_offset;
	x_rot_offset << 0, 0, -1,
					0, 1, 0,
					1, 0, 0;
					
	Eigen::Matrix3d y_rot_offset;
	y_rot_offset << 1, 0, 0,
					0, 0, -1,
					0, 1, 0;
					
	Eigen::Matrix3d z_rot_offset;
	z_rot_offset << 1, 0, 0,
					0, 1, 0,
					0, 0, 1;
					
	Eigen::Matrix4d x_frame;
	x_frame << x_rot_offset, x_pos_offset,
				0, 0, 0, 1;
	Eigen::Matrix4d y_frame;
	y_frame << y_rot_offset, y_pos_offset,
				0, 0, 0, 1;
	Eigen::Matrix4d z_frame;
	z_frame << z_rot_offset, z_pos_offset,
				0, 0, 0, 1;
				
	x_frame = ee_pose*x_frame;
	y_frame = ee_pose*y_frame;
	z_frame = ee_pose*z_frame;
				
	Eigen::Vector3d x_pos = x_frame.block(0,3,3,1);
	Eigen::Vector3d y_pos = y_frame.block(0,3,3,1);
	Eigen::Vector3d z_pos = z_frame.block(0,3,3,1);
	
	Eigen::Matrix3d x_rot = x_frame.block(0,0,3,3);
	Eigen::Matrix3d y_rot = y_frame.block(0,0,3,3);
	Eigen::Matrix3d z_rot = z_frame.block(0,0,3,3);
	
	Eigen::Vector3d angles_x = x_rot.eulerAngles(2,1,0).reverse();
	Eigen::Vector3d angles_y = y_rot.eulerAngles(2,1,0).reverse();
	Eigen::Vector3d angles_z = z_rot.eulerAngles(2,1,0).reverse();
					
					
	
	double roll_x = angles_x(0);
	double pitch_x = angles_x(1);
	double yaw_x = angles_x(2);
	
	double roll_y = angles_y(0);
	double pitch_y = angles_y(1);
	double yaw_y = angles_y(2);
		
	double roll_z = angles_z(0);
	double pitch_z = angles_z(1);
	double yaw_z = angles_z(2);	
	
	// X-Axis
	markerMsgX.set_ns("default");
	markerMsgX.set_id(2);
	markerMsgX.set_action(ignition::msgs::Marker::ADD_MODIFY);
	markerMsgX.set_type(ignition::msgs::Marker::CYLINDER);
		
	ignition::msgs::Set(markerMsgX.mutable_scale(),ignition::math::Vector3d(radius, radius, length));
	ignition::msgs::Set(markerMsgX.mutable_pose(),ignition::math::Pose3d(x_pos(0),x_pos(1),x_pos(2), roll_x, pitch_x, yaw_x));
			
	markerNode.Request("/marker", markerMsgX);
	
	
	// Y-Axis
	markerMsgY.set_ns("default");
	markerMsgY.set_id(3);
	markerMsgY.set_action(ignition::msgs::Marker::ADD_MODIFY);
	markerMsgY.set_type(ignition::msgs::Marker::CYLINDER);
		
	ignition::msgs::Set(markerMsgY.mutable_scale(),ignition::math::Vector3d(radius, radius, length));
	ignition::msgs::Set(markerMsgY.mutable_pose(),ignition::math::Pose3d(y_pos(0),y_pos(1),y_pos(2), roll_y, pitch_y, yaw_y));
			
	markerNode.Request("/marker", markerMsgY);
	
	
	// Z-Axis
	markerMsgZ.set_ns("default");
	markerMsgZ.set_id(4);
	markerMsgZ.set_action(ignition::msgs::Marker::ADD_MODIFY);
	markerMsgZ.set_type(ignition::msgs::Marker::CYLINDER);
		
	ignition::msgs::Set(markerMsgZ.mutable_scale(),ignition::math::Vector3d(radius, radius, length));
	ignition::msgs::Set(markerMsgZ.mutable_pose(),ignition::math::Pose3d(z_pos(0),z_pos(1),z_pos(2), roll_z, pitch_z, yaw_z));
			
	markerNode.Request("/marker", markerMsgZ);
}

void GazeboController::deleteEEFrameVis()
{
	// X-Axis
	markerMsgX.set_ns("default");
	markerMsgX.set_id(2);
	markerMsgX.set_action(ignition::msgs::Marker::DELETE_MARKER);
	markerNode.Request("/marker", markerMsgX);
	
	// Y-Axis
	markerMsgY.set_ns("default");
	markerMsgY.set_id(3);
	markerMsgY.set_action(ignition::msgs::Marker::DELETE_MARKER);
	markerNode.Request("/marker", markerMsgY);
	
	// Z-Axis
	markerMsgZ.set_ns("default");
	markerMsgZ.set_id(4);
	markerMsgZ.set_action(ignition::msgs::Marker::DELETE_MARKER);
	markerNode.Request("/marker", markerMsgZ);
	
}

void GazeboController::createSphere(double x, double y, double z, double roll, double pitch, double yaw, double x_s, double y_s, double z_s)
{
	markerMsgSphere.set_ns("default");
	markerMsgSphere.set_id(1);
	markerMsgSphere.set_action(ignition::msgs::Marker::ADD_MODIFY);
	markerMsgSphere.set_type(ignition::msgs::Marker::SPHERE);
		
	ignition::msgs::Set(markerMsgSphere.mutable_scale(),ignition::math::Vector3d(x_s, y_s, z_s));
	ignition::msgs::Set(markerMsgSphere.mutable_pose(),ignition::math::Pose3d(x, y, z, roll, pitch, yaw));
			
	markerNode.Request("/marker", markerMsgSphere);
}

void GazeboController::drawManipulabilityEllipsoid(Eigen::Matrix4d ee_pose, Eigen::MatrixXd sigma, Eigen::MatrixXd U)
{
	if(U.cols() != 3 || U.rows() != 3)
	{
		std::cout << "Wrong dimensions of U!" << std::endl;
		return;
	}
	
	if(sigma.rows() != 3)
	{
		std::cout << "Wrong dimensions of sigma!" << std::endl;
		return;
	}
	
	Eigen::Matrix3d R = U;
	Eigen::Vector3d s = sigma;
	
	//Make sure that R resembles a rotation matrix
	if((R.col(0).cross(R.col(1))).dot(R.col(2)) < 0)
	{
		R << U.col(1), U.col(0), U.col(2);
		s << sigma(1),
			 sigma(0),
			 sigma(2);
	}
	
	//Transform Rotation to ee_frame
	R = ee_pose.block(0,0,3,3)*R;
	
	//Get roll pitch yaw from R
	double roll = std::atan2(R(2,1),R(2,2));
	double pitch = std::atan2(-R(2,0),std::sqrt(R(2,1)*R(2,1)+R(2,2)*R(2,2)));
	double yaw = std::atan2(R(1,0),R(0,0));
	
	//Check if one of the singular values is below a treshold (important to not scale the sphere's dimensions to zero)
	for(int i = 0; i < s.size(); i++)
	{
		if(s(i) < 0.01)
			s(i) = 0.01;
	}
	
	
	// Draw the sphere
	createSphere(ee_pose(0,3),ee_pose(1,3),ee_pose(2,3),roll,pitch,yaw,s(0),s(1),s(2));
	
}

void GazeboController::deleteManipulabilityEllipsoid()
{
	deleteSphere();
}

void GazeboController::deleteSphere()
{
	markerMsgSphere.set_ns("default");
	markerMsgSphere.set_id(1);
	markerMsgSphere.set_action(ignition::msgs::Marker::DELETE_MARKER);
	markerNode.Request("/marker", markerMsgSphere);
}

bool GazeboController::isFrameValid()
{
	return (m_ee_frame_valid && m_collision_valid);
}
