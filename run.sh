#!/bin/bash

# Set the environment variables
DISPLAY_VAR=$DISPLAY
ROS_DOMAIN_ID=7

# Set the volume mappings
VOLUME_X11=/tmp/.X11-unix/:/tmp/.X11-unix:rw

# Define Docker volumes and environment variables
ROOT=$(dirname "$0")
DOCKER_VOLUMES="
--volume=$VOLUME_X11 \
--volume=$ROOT/app:/app \
--volume=$ROOT/data:/data \
--volume=$ROOT/jetbot_riva_voice:/source \
"
DOCKER_ENV_VARS="
--env DISPLAY=$DISPLAY_VAR \
--env QT_X11_NO_MITSHM=1 \
--env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
"

# check for V4L2 devices
V4L2_DEVICES=""

for i in {0..9}
do
    if [ -a "/dev/video$i" ]; then
        V4L2_DEVICES="$V4L2_DEVICES --device /dev/video$i "
    fi
done

# check for I2C devices
I2C_DEVICES=""

for i in {0..9}
do
    if [ -a "/dev/i2c-$i" ]; then
        I2C_DEVICES="$I2C_DEVICES --device /dev/i2c-$i "
    fi
done

DOCKER_DEVICES="
--device /dev/snd \
--device /dev/bus/usb \
--device=/dev/input \
"
DOCKER_ARGS="${DOCKER_VOLUMES} ${DOCKER_ENV_VARS} ${V4L2_DEVICES} ${I2C_DEVICES} ${DOCKER_DEVICES}"

# Set the docker image
DOCKER_IMAGE=${DOCKER_IMAGE:-jetbot_riva_voice:latest}

# Run the docker command
docker run -it --rm --net host --ipc host \
${DOCKER_ARGS} \
$DOCKER_IMAGE /bin/bash -c "source install/setup.bash && /bin/bash"
