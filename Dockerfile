# change the tag here when we decide the ur5e setup is finished
FROM jdekarske/homestri-ur5e:latest

SHELL ["/bin/bash", "-c"]

# For gui stuff
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=:1
ENV QT_X11_NO_MITSHM=1
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

RUN \
  apt-get update -qq && \
  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri \
  xserver-xorg-video-dummy x11-apps && \
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

COPY src/ src/remote-experiment

COPY dummy-xorg.conf /etc/X11/dummy-xorg.conf 
COPY startdummy-xorg.bash /etc/X11/startdummy-xorg.bash

# Get everything going
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
 && apt-get update -qq \
 && rosdep update \
 && rosdep install --from-path src --ignore-src -y \
 && catkin_make

CMD ["/bin/bash"]
