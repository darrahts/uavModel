
# args
ARG OS_VERSION=14.04

# fundamental 
FROM ubuntu:${OS_VERSION}

#ENV DEBIAN_FRONTEND noninteractive
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

RUN sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable trusty main" > /etc/apt/sources.list.d/drc-latest.list' \
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	wget http://packages.ros.org/ros.key -O - | sudo apt-key add - \
	wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 

RUN sudo apt update && apt-get --no-install-recommends install -y gazebo drcsim \















