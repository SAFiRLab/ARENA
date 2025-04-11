# syntax=docker/dockerfile:1
# check=skip=SecretsUsedInArgOrEnv
# Use NVIDIA CUDA base image
FROM nvidia/cuda:12.6.0-devel-ubuntu22.04

# Set environment variables for non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive

# Update and install necessary tools
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    rapidjson-dev \
    libwebsocketpp-dev \
    libboost-all-dev \
    aptitude \
    nlohmann-json3-dev \
    python3-argcomplete \
    python3-pip && \
    rm -rf /var/lib/apt/lists/*
	

# Install SSH and git
RUN apt-get update && apt-get install -y \
    ssh \
    git

COPY requirements.txt /home/requirements.txt
RUN pip install -r /home/requirements.txt

# Add the ROS 2 GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

# Add the ROS 2 repository
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Update and install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop && \
    rm -rf /var/lib/apt/lists/*

# Set up locale for ROS
RUN apt-get update && apt-get install -y locales && \
    locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt install -y python3-colcon-common-extensions

# Source the ROS 2 setup script in the container's environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

RUN pip install -U rosdep

# Initialize rosdep
RUN rosdep init && rosdep update

# Create ROS 2 workspace
RUN mkdir -p /home/dev_ws/src

# Clone ROS repositories
RUN git clone https://github.com/facontidavide/rosx_introspection.git /home/dev_ws/src/rosx_introspection
RUN git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git /home/dev_ws/src/ros_tcp_endpoint -b ROS2v0.7.0
RUN apt-get update && apt install -y ros-humble-foxglove-bridge

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && cd /home/dev_ws && colcon build"

# Set workspace environment in the bashrc
RUN echo "source /home/dev_ws/install/setup.bash" >> ~/.bashrc

# Install additional ROS 2 packages
RUN apt-get update && apt-get install -y ros-humble-rosbag2-storage-mcap
#RUN mkdir home/ARENA
#WORKDIR /home/ARENA

#RUN mkdir -p /home/dev_ws/src
#COPY simoffroad/Sim_2_Off_Road_ROS2_Msg/ /home/dev_ws/src


# Build the custom message workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.sh && \
    cd /home/dev_ws/src && \
    ros2 pkg create --build-type ament_cmake arena_msgs && \
    cd /home/dev_ws && \
    colcon build"

RUN /bin/bash -c "rm -r /home/dev_ws/src/arena_msgs/*"

# Add the new workspace setup to bashrc
RUN echo "source /home/dev_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /home/dev_ws

# Set the entry point for the container
CMD ["bash"]