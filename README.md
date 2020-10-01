# HOMESTRI-remote-experiment
To start with the local gui:

`./gui-docker -it jdekarske/homestri-remote-experiment:latest`

Include `-p 9090:9090` to access from roslibjs.

To start the environment:

`roslaunch experiment_world mainexperiment.launch`

Sometimes the controllers throw a bunch of errors and the robot won't move, I think this is because gazebo takes a while to start the first time. just kill it and restart.
