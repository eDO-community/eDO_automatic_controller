FROM moveit/moveit:noetic-source as stop

ENV DEBIAN_FRONTEND=noninteractive

# install ros package dependencies
RUN apt-get update && \
    apt-get install -y \
      git \
    	ros-noetic-ros-control \
    	ros-noetic-ros-controllers \
    	ros-noetic-gazebo-ros \
      ros-noetic-gazebo-ros-control \
    && rm -rf /var/lib/apt/lists/*

# clone ros package repo
ENV WS_EDO_COMMUNITY_SIMULATOR /root/ws_simulator
RUN mkdir -p $WS_EDO_COMMUNITY_SIMULATOR/src
WORKDIR $WS_EDO_COMMUNITY_SIMULATOR

ENV REPOSITORY_URL "https://github.com/Pro/edo_gripper" \
      "https://github.com/Pro/edo_gazebo"\
      "https://github.com/Pro/eDO_description"\
      "https://github.com/Pro/edo_gripper_moveit"

ADD https://api.github.com/repos/Pro/edo_gazebo/git/refs/heads/master /root/edo_gazebo.json
RUN echo ${REPOSITORY_URL} | xargs -n1 | xargs -I{} git -C src clone {}

# install ros package dependencies
RUN rosdep update && \
    rosdep install -y \
      --from-paths \
        /opt/ros/noetic/include \
      --ignore-src \
    && rm -rf /var/lib/apt/lists/

ENV ROS_DIRECTORY /opt/ros/$ROS_DISTRO

# moveit packages are not installed by default, but in a specific catkin workspace
# build them and use this directory as an overlay of our catkin directory
ENV MOVEIT_DIRECTORY /root/ws_moveit

WORKDIR $MOVEIT_DIRECTORY

RUN catkin config \
      --extend $ROS_DIRECTORY && \
    catkin build

# build catkin directory
WORKDIR $WS_EDO_COMMUNITY_SIMULATOR

RUN catkin config \
      --extend $MOVEIT_DIRECTORY/install && \
    catkin build

# source ros package from entrypoint
RUN sed --in-place --expression \
      '$isource "$WS_EDO_COMMUNITY_SIMULATOR/devel/setup.bash"' \
      /ros_entrypoint.sh && \
    sed --in-place --expression \
      '$isource "$WS_EDO_COMMUNITY_SIMULATOR/devel/setup.bash"' \
      /root/.bashrc

RUN apt-get update && \
    apt-get install -y python3-venv \
      python3-pip && \
    pip3 install -U numpy && \
    pip3 install numpy-quaternion

WORKDIR /opt/ros_ws/src

CMD ["/bin/bash", "-c", "tail -f /dev/null"]
# NAME_CONTAINER=edo_gazebo
# docker run --privileged --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --net=host -v /tmp/.X11-unix:/tmp/.X11-unix $NAME_CONTAINER rqt
#"roslaunch edo_gazebo edo_gripper.launch & && "\
#        "roslaunch edo_gripper_moveit edo_moveit_planning_execution.launch" \
#          "__ns:=edo sim:=true & &&" \
#        "roslaunch edo_gripper_moveit moveit_rviz.launch __ns:=edo config:=true"]
#roslaunch edo_gazebo edo_gripper.launch &
#roslaunch edo_gripper_moveit edo_moveit_planning_execution.launch __ns:=edo sim:=true &
#roslaunch edo_gripper_moveit moveit_rviz.launch __ns:=edo config:=true
