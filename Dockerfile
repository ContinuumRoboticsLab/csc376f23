#FROM  misterkoz/ros-melodic-with-vnc-and-gazebo
FROM tiryoh/ros-desktop-vnc

ARG USERNAME=ubuntu


COPY env_files/bash_profile /home/ubuntu/.bashrc
COPY env_files/vimrc /home/ubuntu/.vimrc

# the startup script needs to run as root
USER root
