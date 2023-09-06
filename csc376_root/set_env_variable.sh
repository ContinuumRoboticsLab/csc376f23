echo "export ROS_HOSTNAME=localhost" >> ~/.bashrc
echo "export ROS_MASTER_URI=http://localhost:11311" >> ~/.bashrc
# echo "export WORKSPACE=/home/ubuntu/catkin_ws" >> ~/.bashrc

echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:$PWD/csc376-assignment0/build" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:$PWD/csc376-assignment1/build" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:$PWD/csc376-assignment2/build" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:$PWD/csc376-assignment3/build" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=\${GAZEBO_MODEL_PATH}:$PWD/models" >> ~/.bashrc
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:$PWD/models" >> ~/.bashrc


echo "alias vim=vi" >> ~/.bashrc
