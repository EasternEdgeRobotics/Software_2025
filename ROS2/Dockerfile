FROM ros:jazzy-ros-base-noble

RUN apt-get update && apt-get install -y python3 python3-pip ros-humble-rosbridge-server ffmpeg libsm6 libxext6 dos2unix
RUN pip3 install sqlalchemy RPi.GPIO smbus2 adafruit-circuitpython-bno055 adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit opencv-python

WORKDIR /app
COPY ROS_workspace/src /app/src
COPY ROS_workspace/docker_entry_script.sh /app

ARG CACHEBUST=1
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
RUN echo "source /app/install/setup.bash" >> /etc/bash.bashrc
RUN dos2unix /app/docker_entrypoint.sh

ENTRYPOINT ["bash", "/app/docker_entry_script.sh" ]
