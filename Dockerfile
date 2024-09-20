
# Use an argument for the base image
# ARG BASE_IMAGE=dustynv/ros:humble-llm-r36.3.0
ARG BASE_IMAGE=dustynv/nano_llm:humble-r36.3.0

FROM ${BASE_IMAGE}

# ARG ROS2_SETUP=/opt/ros/humble/install/setup.bash
ARG ROS2_SETUP=/ros2_workspace/install/setup.bash

# ENV key=value
ENV ROS_DISTRO=humble
ENV SHELL=/bin/bash

# Set up the ROS2 workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws/src

# Copy the requirements.txt file
COPY requirements.txt ./requirements.txt

# Copy the entrypoint script into the image
# COPY /ros_entrypoint.sh /

# Copy your ROS2 packages into the workspace
COPY ./jetbot_riva_voice .

# Update the package list and install vi
RUN apt-get update && apt-get install -y vim

# Install any necessary dependencies
RUN pip3 install --no-cache-dir --verbose -r requirements.txt

# {BASE_IMAGE} will setup the ROS2 ENTRYPOINT
ENTRYPOINT [ "/ros_entrypoint.sh" ]

# Build the workspace
WORKDIR /ros2_ws
RUN source ${ROS2_SETUP} && \
    colcon build

CMD ["/bin/bash"]
