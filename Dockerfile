# change the tag here when we decide the ur5e setup is finished
FROM jdekarske/homestri-ur5e:latest

SHELL ["/bin/bash", "-c"]

# Setup environment
WORKDIR /catkin_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash

# Get ROSPlan stuff
RUN apt-get update -qq && apt-get install -y \
 flex bison freeglut3-dev libbdd-dev python-catkin-tools ros-$ROS_DISTRO-tf2-bullet \
 && git clone https://github.com/KCL-Planning/rosplan ./src/rosplan

# Mongodb (ROSPlan dep) stuff
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 68818C72E52529D4 \
 && echo "deb http://repo.mongodb.org/apt/ubuntu bionic/mongodb-org/4.0 multiverse" >> /etc/apt/sources.list.d/mongodb-org-4.0.list \
 && apt-get update -qq && apt-get install -y \
 mongodb-org \
 && git clone -b melodic-devel https://github.com/strands-project/mongodb_store.git --single-branch ./src/mongodb_store
