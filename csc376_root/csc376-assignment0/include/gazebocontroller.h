#pragma once

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <gazebo/gazebo_client.hh>

#include <ignition/transport.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>


#include <Eigen/Dense>
#include <Eigen/Geometry>


/// \brief A class that serves as an easy interface to the Gazebo Simulation
class GazeboController
{
    
    public:
		
		//Constructor and Destructor
		GazeboController();
		~GazeboController();
		
		// Function to set the configuration of the simulated robot
		// Input: q - 7x1 Matrix containing acutation values for each joint in rad
		void setRobotConfiguration(Eigen::MatrixXd q);
		
		// Function to obtain the current end-effector frame of the simulated robot
		// Output: 4x4 Matrix describing the frame of the simulated robot end-effector
		Eigen::Matrix4d getCurrentRobotEEFrame();
		
		// Function to obtain the current configuration of the simulated robot
		// Output: 7x1 Matrix containing the current acutation values of the simulated robot for each joint in rad
		Eigen::MatrixXd getCurrentRobotConfiguration();
		
		// Function to draw a colored frame in simulation
		// Input: ee_pose - 4x4 Matrix which specifies the desired pose of the drawn frame (which should be the calculated robot end-effector pose)
	    void moveEEFrameVis(Eigen::Matrix4d ee_pose);
	    
		// Function to remove the colored frame from simulation
		void deleteEEFrameVis();
		
		// Functions to draw a manipulability ellipsoid in simulation
		// Input: ee_pose - 4x4 Matrix describing the pose of the ellipsoids center (which should be the calculated robot end-effector pose)
		// Input: sigma - 3x1 column Vector of singular values of translation part of Body Jacobian
		// Input: U - 3x3 Matrix containing the singular vectors of translation part of Body Jacobian as columns (nth column corresponds to nth singular value in sigma)
		void drawManipulabilityEllipsoid(Eigen::Matrix4d ee_pose, Eigen::MatrixXd sigma, Eigen::MatrixXd U);
		
		// Functions to remove the manipulability ellipsoid from simulation
		void deleteManipulabilityEllipsoid();
		
		// Function that returns true if the current end-effector frame is valid
		// When the configuration of the robot is changed, wait for this function to turn true to make sure the robot finished updating in simulation
		bool isFrameValid();
		
		// Function to obtain the current collision state of the simulated robot
		// Output: Bool value indicating whether the robot is in collision (true) or not (false)
		bool getCollisionState();

     
	private:
		
		//Private "internal" functions and variables
		
		
		void OnEEPoseMsg(ConstPosePtr &_msg);
		void OnCollisionMsg(ConstJointPtr &_msg);

		
		Eigen::MatrixXd m_robot_config;
		Eigen::Matrix4d m_robot_ee_frame;
	
		bool m_ee_frame_valid;
		bool m_collision_valid;
		bool m_collision;

		
		gazebo::transport::NodePtr mp_gazebo_node;

		gazebo::transport::PublisherPtr mp_joint_pos_publisher;  
		
		gazebo::transport::SubscriberPtr mp_ee_pose_subscriber; 
		gazebo::transport::SubscriberPtr mp_collision_subscriber; 

		
		ignition::msgs::Material *matMsgSphere;
		ignition::msgs::Marker markerMsgSphere;
		
		ignition::msgs::Material *matMsgX;
		ignition::msgs::Marker markerMsgX;
		ignition::msgs::Material *matMsgY;
		ignition::msgs::Marker markerMsgY;
		ignition::msgs::Material *matMsgZ;
		ignition::msgs::Marker markerMsgZ;
		
		ignition::transport::Node markerNode;
		
		
		
		void createSphere(double x, double y, double z, double roll, double pitch, double yaw, double x_s, double y_s, double z_s);
		void deleteSphere();
};

