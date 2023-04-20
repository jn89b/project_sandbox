#example to build dockerfile
FROM osrf/ros:humble-desktop AS base
SHELL [ "/bin/bash" , "-c" ]

# Install basic apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 git libcanberra-gtk-module libcanberra-gtk3-module libqt5svg5-dev \
 python3-pip python3-opencv python3-tk python3-pyqt5.qtwebengine

# Install additional Python modules
RUN pip3 install matplotlib transforms3d install pymavlink casadi 

# Install additional ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
 ros-humble-cartographer \
 ros-humble-cartographer-ros \
 ros-humble-gazebo-ros \
 ros-humble-navigation2 \
 ros-humble-nav2-bringup \
 ros-humble-rmw-cyclonedds-cpp

# Use Cyclone DDS as middleware
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Downgrade mesa packages because of Ubuntu bug
# https://bugs.launchpad.net/ubuntu/+source/mesa/+bug/2004649
RUN apt-get update && apt-get install -q -y --allow-downgrades --no-install-recommends \
    libegl-mesa0=22.0.1-1ubuntu2 \
    libgbm1=22.0.1-1ubuntu2 \
    libgbm-dev=22.0.1-1ubuntu2 \
    libgl1-mesa-dri=22.0.1-1ubuntu2 \
    libglapi-mesa=22.0.1-1ubuntu2 \
    libglx-mesa0=22.0.1-1ubuntu2 

# Create Colcon workspace with external dependencies
RUN mkdir -p /ros2_ws/src
# WORKDIR /ros2_ws/src

# Build the base Colcon workspace, installing any extra missed dependencies.
WORKDIR /ros2_ws
RUN apt-get install -y python3-vcstool python3-rosinstall-generator python3-osrf-pycommon \
    && rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos \
    && rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos \
    && vcs import src < /tmp/mavlink.repos \
    && vcs import src < /tmp/mavros.repos \
    && apt-get update -y \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y \
    && ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

# RUN source /opt/ros/humble/setup.bash \
#     && colcon build --symlink-install 

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]

#######################################################
# Overlay Development Image #
#######################################################
FROM base AS develop

#Create an overlay workspace
RUN mkdir -p /develop_ws/src \
    && echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

WORKDIR /develop_ws/src

COPY dependencies.repos ./
RUN vcs import < dependencies.repos 

RUN cd ../ \
    && colcon build --symlink-install \
    && source install/setup.bash 

 # Set up the entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT [ "/entrypoint.sh" ]







