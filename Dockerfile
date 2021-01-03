# change the tag here when we decide the ur5e setup is finished
FROM jdekarske/homestri-ur5e:latest

SHELL ["/bin/bash", "-c"]

# For gui stuff (I think)
RUN \
  apt-get update -qq && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
  rm -rf /var/lib/apt/lists/*

# Setup environment
WORKDIR /catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash

#Rosbridge for web viewer
RUN apt-get update -qq && apt-get install -y \
 ros-melodic-rosbridge-server

# Get ROSPlan stuff
RUN apt-get update -qq && apt-get install -y \
 flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet \
 && git clone https://github.com/KCL-Planning/rosplan ./src/rosplan

##########################################

COPY src/experiment_world /catkin_ws/src/experiment_world
COPY src/target_pose /catkin_ws/src/target_pose
COPY src/spawn_objects /catkin_ws/src/spawn_objects

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin_make

# For rosbridge
EXPOSE 9090

CMD ["/bin/bash"]
# ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/docker-entrypoint.sh && roslaunch moveit_config demo.launch"]
# run this from the git repo: $ ./gui-docker -it -p 9090:9090 -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-remote-experiment:latest
# ./gui-docker -it -p 9090:9090 -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel --cap-add=SYS_PTRACE --security-opt seccomp=unconfined --security-opt apparmor=unconfined jdekarske/homestri-remote-experiment:latest