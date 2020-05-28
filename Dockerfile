FROM osrf/ros:dashing-desktop

RUN apt-get update && \
    apt-get install python-pip python3-pip -y && \
    pip install --upgrade pip
RUN apt-get install -y ros-dashing-rosbridge-server

COPY . /root/dev_ws/src/rowma_ros2

SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/dashing/setup.bash && \
    cd /root/dev_ws && \
    colcon build && \
    source /opt/ros/dashing/setup.bash && source ~/dev_ws/install/local_setup.bash \
    pip3 install "python-socketio[client]"

CMD ["ros2", "run", "rowma_ros2", "rowma"]
