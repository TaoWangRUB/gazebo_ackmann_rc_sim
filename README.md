This repo is aiming to use ROS2 and gazebo to build up simulation environment to simulate an rc with ackmann steering navigating in a surrounded area.

**Dependencies**
1. ROS2 Humble
2. Gazebo Harmonic

*Note*: ROS2 foxy can also be used to communicate with Gazebo. Following setup has to be done. Since Gazebo Harmonic (gz8) is only compatible with ROS2 Humble and it is only natively supported under Ubuntu22, therefore a docker environment is necessary. ROS2 Humble and Gazebo Harmonic can also manually recompiled under Ubuntu20, but it requires too many manual works. Therefore docker solution is needed.

**Using Docker**

If native Ubuntu22 is used, please skip the following step.
otherwise if you're using docker, first build the proper docker container for Ubuntu22 + ROS2 Humble + Gazebo Harmonic under project root

```
docker build -t ros2-gazebo-harmonic .
xhost +local:root  # Allow container to use your X display
docker run --rm -it --env="DISPLAY=${DISPLAY}" \
    --runtime=nvidia \
    --gpus all \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/workspace" \
    --device /dev/dri  \
    --net=host \
    ros2-gazebo-harmonic \
    bash
```

**Compiling the Code**

In the root of the project, build ROS2 pkg using

```
colcon build && source install/setup.bash
```

**Launch gz simulation**

```
ros2 launch robot_description robot_description.py
```

**Control the RC from ROS2**
```
ros2 topic pub cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 1.}}" -1
```

*Note*: if ROS2 command is sent out from different version of ROS2, such as foxy, the underlying middleware of clonedds should be used for both (foxy and humble). (By default fastdds is used, which has problem for communication between humble and foxy)
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```