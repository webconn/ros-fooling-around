FROM ros:melodic

MAINTAINER Nikita Maslov <wkernelteam@gmail.com>

LABEL com.nvidia.volumes.needed="nvidia_driver"

RUN rm /etc/apt/apt.conf.d/docker-clean

RUN apt-get update && apt-get -y install nano vim bash-completion \
    qt5-default libgl1-mesa-glx libgl1-mesa-dri \
    ros-melodic-rviz ros-melodic-tf ros-melodic-tf2 ros-melodic-urdf ros-melodic-xacro \
    ros-melodic-robot-state-publisher ros-melodic-joint-state-publisher ros-melodic-joint-state-controller \
    ros-melodic-cv-bridge ros-melodic-tf-conversions ros-melodic-eigen-conversions ros-melodic-roslint \
    libcairomm-1.0-dev && rm -rf /tmp/* /var/tmp/*

COPY bashrc entrypoint.sh /docker/
