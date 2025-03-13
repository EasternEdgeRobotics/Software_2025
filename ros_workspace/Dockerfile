FROM ros:jazzy-ros-base-noble

# Specify ROV name
ARG ROV_NAME=waterwitch

# Specify whether this is being built for sim (1) or real ROV (0)
ARG DEV_MODE=false

ENV ROV_NAME=${ROV_NAME}
ENV DEV_MODE=${DEV_MODE}

RUN if [ "$DEV_MODE" = "false" ]; then \
    # For real ROV (compiles on Raspberry Pi)
    apt-get update && apt-get install -y python3 python3-pip ros-jazzy-rosbridge-server libsm6 libxext6 dos2unix i2c-tools && \
    
    # Python packages required only for Beaumont
    if [ "$ROV_NAME" = "beaumont" ]; then \
        pip3 install --break-system-packages sqlalchemy RPi.GPIO smbus2 adafruit-circuitpython-bno055 adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit opencv-python; \
    fi \
else \
    # For development (to interface with Simulator)
    # TODO: (Zaid) I don't think all those packages are required for simulation
    apt-get update && apt-get install -y python3 python3-pip ros-jazzy-rosbridge-server ros-jazzy-ament-cmake ros-jazzy-web-video-server ffmpeg libsm6 libxext6 dos2unix libglfw3 libglfw3-dev && \
    apt install -y python3-sqlalchemy python3-opencv; \
fi

WORKDIR /app
COPY src /app/src
COPY docker_entry_script.sh /app

ARG CACHEBUST=1

# Only compile required packages
RUN if [ "$ROV_NAME" = "beaumont" ]; then \
    . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-ignore waterwitch_backend waterwitch_frontend; \
else \
    . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --packages-ignore beaumont_backend waterwitch_frontend; \
fi

RUN echo 'source /app/install/setup.bash $ROV_NAME $DEV_MODE' >> /etc/bash.bashrc
RUN dos2unix /app/docker_entry_script.sh

ENTRYPOINT bash /app/docker_entry_script.sh "$ROV_NAME" "$DEV_MODE"
