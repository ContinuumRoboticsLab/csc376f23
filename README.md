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
    * Docker and Docker compose will be installed when installing Docker Desktop. Install from [here](https://www.docker.com/products/docker-desktop/). 
        * run the docker desktop and ensure docker is able to start and run without any issues
            * A wsl error might occur. follow instructions it provides to install wsl.
    * Install make using choco following commands [here](https://chocolatey.org/install) and then run in PowerShell or command prompt **as an Adminisitrator**,
    ```
    choco install make
    ```
    * Download and Install VNC viewer from [here](https://www.realvnc.com/en/connect/download/viewer/windows/)





## Usage

### Build and run docker container and VNC server 

* Ubuntu 
    * run, 
    ```
    cd  <path to this repo>/csc376f22
    sudo make up
    ```
* macOS 
    * run, 
    ```
    cd  <path to this repo>/csc376f22
    make up
    ```
* Windows 
    * First ensure docker-desktop is running 
    * Then open a powershell terminal in the location of this repo csc376f22 **as an Administrator**
    * now run,
    ```
    make up
    ```
    * Once the compilation is done, open the vnc viewer and type in the following URL to access the Ubuntu docker environment's desktop.
    ```
    127.0.0.1:5900
    ```

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
   

## Notes
* In addition to running the simulation environment on your own machine using the provided Docker environment, students can also use their UTORid to log in to the Alienware machines in the teaching lab **MN3110**, which runs Ubuntu and has Gazebo preinstalled. For this, simply copy the folder **csc376f22** from the above repository into your **home directory**. The simulation environment's starter code is then contained in the folder **csc376_root**. You can directly modify and compile this code  - there is no need to set up and use the Docker environment, when working on these machines.
* All changes made in the local machine in the **csc376_root** directory will be reflected inside the docker container. So make use of this directory for code development.
* You will find individual folders for each assignment of the course in this **csc376_root**. For now, starter code for Assignment 0 (content of Practical 1) is provided. The starter code for Assignments 1-3 will be released via MarkUs once the corresponding assignment goes online.
* Assignment 0 is the content of Practical 1 and is used as an introuction to the simulation framework. Despite its name, **Assignment 0 is not a graded assignment** for CSC376.

## References for docker environment

* [richardw05/mybot_ws](https://github.com/richardw05/mybot_ws)
* [mrkoz/mybot_ws](https://github.com/mrkoz/mybot_ws)
