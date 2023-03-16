# Project Sandbox
This repository provides an interface with ROS2, Unity, and Ardupilot to simulate drone interactions in the Unity environment

## Setup
First build the docker images with the the following command
```
docker compose build
```

## Basic Usage
To run a docker container run the developer container 

```
docker compose up develop
```

Once that is running run a separate container using:
```
docker exec -it project_sandbox-develop-1 bash
```

## Demo


## Helpful Shell Scripts/Commands

### Cleaning Dangling Containers
If you ever notice that you have a lot of dangling Docker containers run the following shell command
```
./clean_docker_out.sh
```
This will prune and remove any containers that have <None> tag

### GUI with Docker
```
xhost +local:docker 
```
