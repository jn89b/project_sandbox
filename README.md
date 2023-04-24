# Project Sandbox
This repository provides an interface with ROS2, Unity, and Ardupilot to simulate drone interactions in the Unity environment

## Setup
### Unity Installation 
Install Unity, afterwards please follow the attached link to install the **ROS-TCP-Connector** https://github.com/Unity-Technologies/ROS-TCP-Connector

After you have installed the container build the docker images with the the following command
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
## Extending Volumes from your host
In the **docker-compose.yaml** to extend volumes from your host directory add the folder extension underneath the **volumes**
```yaml
 # develop image containing the example source code.
  develop:
    extends: base
    image: sandbox:develop
    build:
      context: .
      dockerfile: docker/Dockerfile
      target: develop
    # Interactive shell
    stdin_open: true
    tty: true
    # Networking and IPC for ROS 2
    network_mode: host
    ipc: host
    #port access from the container to the host
    ports:
    - "10000:10000"
    environment:
    - DISPLAY=${DISPLAY}
    - QT_X11_NO_MITSHM=1
    - NVIDIA_DRIVER_CAPABILITIES=all
    volumes:
    - ./drone_ros:/develop_ws/src/drone_ros:rw
    - ./mpc_ros:/develop_ws/src/mpc_ros:rw
    - ./drone_interfaces:/develop_ws/src/drone_interfaces:rw
    - ./unity_robotics_demo:/develop_ws/src/unity_robotics_demo:rw
    - ./unity_robotics_demo_msgs:/develop_ws/src/unity_robotics_demo_msgs:rw
    - ./folder_directory_name:/develop_ws/src/folder_directory_name
    command: sleep infinity
    logging:
      options:
        max-size: 50m # maximum size of log file before rotation
```

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

 
