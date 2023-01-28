#!/usr/bin/env bash

# TODO this is doing something weird
# claiming unbound variable ROS_URI
# I think this should go after source ros or use 'set' that is okay with unbound variables
# set -o nounset

# This program's basename.
_ME="$(basename "${0}")"

_source_ros() {
    source /catkin_ws/devel/setup.bash
}

_start() {
    _source_ros
    roslaunch experiment_world main_experiment.launch
}

_kill() {
    _source_ros
    rosnode kill -a && pkill -f '(ros)|(gzserver)'
}

_rosbag() {
    _source_ros
    rosbag record --duration $1 -O /catkin_ws/rosbags/"$2"-$(date +'%Y%m%d%H%M%S').bag -a -x '.*camera.*' 
}

_status() {
    _source_ros
    rostopic list
}

_main() {
    case $1 in
      -s|--start) _start;;
      -k|--kill) _kill;;
      -b|--rosbag) _rosbag $2 $3;; # duration then workerID
      -t|--status) _status;;
      *) "Unknown parameter passed: $1";;
    esac;
}

# Call `_main` after everything has been defined.
_main "$@"

