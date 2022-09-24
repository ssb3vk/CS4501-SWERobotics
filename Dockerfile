FROM ubuntu:18.04

# Set the work directory 
WORKDIR /root

# Minimal setup
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    lsb-release \
    locales \
    gnupg2 \
    gfortran \
    htop 

# Stop questions about geography
ARG DEBIAN_FRONTEND=noninteractive
RUN dpkg-reconfigure locales

# Prepare ROS installation
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS melodic
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-melodic-desktop-full

# Install additional packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-melodic-imu-filter-madgwick \
    ros-melodic-mav-msgs \
    libsdl-image1.2-dev \
    python-catkin-tools \
    python-tk \
    python-pip \
	git

# Setup ROS dep
RUN apt-get install -y --no-install-recommends python-rosdep
RUN rosdep init \
 && rosdep fix-permissions \
 && rosdep update

RUN python -m pip install --upgrade pip && \
    python -m pip install \
    tensorflow==1.5 \
    keras==2.1.4 \
    tqdm==4.64.1 \
    sklearn==0.0 \
    h5py==2.10.0

# Source ROS
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

#Setup python\r alias
RUN ln /usr/bin/python $(printf "/usr/bin/python\r")
