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

# 3. Install TurtleBot3 and others
RUN apt-get update && \
    apt-get purge -y ros-humble-ros-gz* && \
    apt-get install -y \
    #ros-humble-turtlebot3* \
    ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes \
    ros-humble-realsense2* \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# 4. Workspace
RUN mkdir -p /workspace
WORKDIR /workspace

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

