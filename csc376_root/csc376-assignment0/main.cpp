#include <gazebocontroller.h>
#include <pandarobotmodel.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <algorithm>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

// Main code of the assignment framework
int main(int _argc, char **_argv)
{
	
	// Create Gazebo controller and Panda robot model objects
	GazeboController controller;
	PandaRobotModel robot_model;

	
	//Create a set of angles for each joint
	Eigen::Matrix<double,7,1> q;
	
	//-----Your code goes here-----



	//-----Your code ends here-----
	
	// Move robot to initial configuration (all angles are zero)
	q.setZero();
	q(3) = -0.07;
	controller.setRobotConfiguration(q);
	
	//Make sure the robot moved in simulation and the EE frame updated
	while(!controller.isFrameValid())
	{
	}
    
    
    return 0;
}
