# CSC376 Simulation Environment

Modified from : [mrkoz/mybot_ws](https://github.com/mrkoz/mybot_ws)

## Introduction

This repository contains a docker container that hosts a the simulation environment for "CSC376: Fundamental of Robotics" at the University of Toronto. It runs Ubuntu 20.04 with Gazebo (11.10.2), Eigen (3.3.7) and CMake (3.16.3). Upon building and running the container, one can access the simulation environment's desktop with a VNC viewer. 

## Dependencies 

In order to run this docker container the host machines (Ubuntu/macOS/Windows) should have 
* docker 
* docker-compose
* A vnc viewer (tested tigervnc)
* cmake (tested 3.16.3)




### Install Dependencies

* Ubuntu 
    * Check CMake installation or else install it 
    ```
    cmake --version 
    # if this throws a cmake not found error 
    sudo apt-get install cmake 
    ```
    * Install docker 
    ```
    sudo apt-get install docker.io
    sudo docker run hello-world # tests docker installation
    ```
    * Install docker-compose
    ```
    sudo apt-get install docker-compose
    ```
    * Install a vncviewer eg: tigervnc 
    ```
    sudo apt-get install tigervnc-viewer
    ```

* macOS
    * Install Cmake 
    ```
    brew install cmake 
    ```
    * Docker installation-
    
        Follow instructions [here](https://runnable.com/docker/install-docker-on-macos) to install docker on Mac. To verify installation run the commands below and you should not get any error.  
    ```
    docker --version 
    docker-machine --version 
    docker-compose --version
    ```

* Windows 
    * Windows installation instructions comes here




## Usage

### Build and run docker container and VNC server 

* Ubuntu 
    * run, 
    ```
    cd  <path to this repo>/csc376-dev-env
    sudo make up
    ```
* macOS 
    * run, 
    ```
    cd  <path to this repo>/csc376-dev-env
    make up
    ```
* Windows 
    * windows installation comes here  

### Run VNC client to access dev environment 
After running the docker container using ```make up``` command, run the vnc viewer to access the development environment desktop. In a new terminal run, 

* Ubuntu 
    ```
    vncviewer 127.0.0.1:5900
    ```
    The password is **ubuntu** 
* macOS 
    ```
    open vnc://127.0.0.1:5900
    ```


### Once VNC client is running and you can access the desktop 

* (Ubuntu/macOS/Windows) Common Instructions
    1. one can access the terminal by pressing the home button(button left corner in VNC desktop) -> System Tools -> LXTerminal
    
    2. To test build environment 

    *   Inside vnc (make sure to replace X with a number from 0-3, depending on the assignment),  
        ```
        cd /home/ubuntu/csc376/csc376-assignmentX
        mkdir build 
        cd build 
        cmake ..
        make 
        ```
    * run simulation of the robot and its world in gazebo (make sure to replace X with a number from 0-3, depending on the assignment),
        ```
        cd /home/ubuntu/csc376/csc376-assignmentX
        gazebo csc376_assignment.world --verbose
        ```
    * **Open a new terminal** and run your code (make sure to replace X with a number from 0-3, depending on the assignment),
        ```
        cd /home/ubuntu/csc376/csc376-assignmentX/build/
        ./csc376-assignment
        ```
   

## Note
* All changes made in the local machine in the **csc376_root** directory will be reflected inside the docker container. So make use of this directory for code development.

## References for docker environment

* [richardw05/mybot_ws](https://github.com/richardw05/mybot_ws)
* [mrkoz/mybot_ws](https://github.com/mrkoz/mybot_ws)
