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
    ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes ros-humble-gz-ros2-control ros-humble-ign-ros2-control-demos ros-humble-ros2controlcli\
    ros-humble-realsense2* \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    ros-humble-robot-localization \
    ros-humble-imu-transformer \
    ros-humble-joint-state-publisher-gui \
    ros-humble-plotjuggler-ros \
    ros-humble-rmw-cyclonedds-cpp && \
    rm -rf /var/lib/apt/lists/*

# 4. Workspace
RUN mkdir -p /workspace
RUN useradd -m -u 1000 px4 && \
    echo "px4 ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER px4
ENV HOME=/home/px4
WORKDIR /workspace

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

