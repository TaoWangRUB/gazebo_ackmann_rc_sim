FROM osrf/ros:humble-desktop-full

# 1. Install system dependencies
RUN apt-get update && apt-get install -y \
    lsb-release \
    wget \
    gnupg2 \
    curl \
    locales \
    sudo
    
# 2. Add Gazebo repository
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
    http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# 3. Install Gazebo Harmonic
RUN sudo apt-get update && \
    sudo apt-get install -y \
    gz-harmonic \
    ros-humble-ros-gzharmonic \
    ros-humble-rmw-cyclonedds-cpp && \
    sudo rm -rf /var/lib/apt/lists/*

# 4. Set up environment
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
RUN echo "source /usr/share/gz/gz-sim/setup.sh" >> ~/.bashrc
