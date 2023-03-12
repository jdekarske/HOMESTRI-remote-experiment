# HOMESTRI-remote-experiment

This repository is the robotic simulation component of a human-robot interaction experiment.

The other components for this experiment are:
- [GUI](https://github.com/jdekarske/HOMESTRI-dragdrop)
- [Robot Base](https://github.com/jdekarske/HOMESTRI-UR5e)
- Network Configuration (Currently private for security)
- Container Management (Currently private for security)

## Install
This software was designed for portability and concurrency using a microservice architecture. However, since simulated camera rendering is required, you must use the nvidia runtime.

1. Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker) noting the requirements.

2. Download the latest experiment image: `docker pull jdekarske/HOMESTRI-remote-experiment:latest`

## Run
Ideally the associated Container Management software is used to distribute human subjects to container environments. For manually managed containers there are two steps:

1. Start the container `docker run -name ros -p 9090:9090 jdekarske/HOMESTRI-remote-experiment`
2. Start the experiment `docker exec --it ros /catkin_ws/src/remote-experiment/scripts/manager.sh -l`

## Development
The ideal development environment for this project depends on your editor preference. If you are using VSCode, use the [development container](https://code.visualstudio.com/docs/devcontainers/containers) workflow. Otherwise, you can run the container with a volume mount of the src directory: `docker run --it -v $PWD/src/:/catkin_ws/src/remote-experiment/ jdekarske/HOMESTRI-remote-experiment bash`
