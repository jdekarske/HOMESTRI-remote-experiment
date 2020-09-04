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

##########################################

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin_make

CMD ["/bin/bash"]
# ENTRYPOINT ["/bin/bash", "-c", "source /catkin_ws/docker-entrypoint.sh && roslaunch moveit_config demo.launch"]
# run this from the git repo: $ ./gui-docker -it -v $PWD/experimentdevel:/catkin_ws/src/experimentdevel jdekarske/homestri-remote-experiment:latest