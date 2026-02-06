
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


# ---------------------------------------------------------------------------
# WORKAROUND: Fix broken ROS2 apt repository in dustynv/nano_llm:humble-r36.3.0
#
# The base image contains a malformed /etc/apt/sources.list.d/ros2.list and an
# expired ROS GPG key (F42ED6FBAB17C654). This causes all `apt-get update`
# commands to fail with:
#   "Malformed entry" or "EXPKEYSIG ... Open Robotics"
#
# This block removes the broken repo file, installs a fresh ROS keyring, and
# recreates a valid ROS2 repository entry so apt can function normally.
# ---------------------------------------------------------------------------

# Remove the broken ROS2 repo file entirely
RUN rm -f /etc/apt/sources.list.d/ros2.list

# Install curl + gnupg
RUN apt-get update && apt-get install -y curl gnupg2

# Install the new ROS2 keyring (Docker-safe)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --batch --yes --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Recreate the ROS2 repo with the new key
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list


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
