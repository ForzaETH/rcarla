ARG ROS_DISTRO=noetic

FROM carla-ros-bridge:${ROS_DISTRO}

ARG ROS_DISTRO=noetic

RUN mkdir -p /opt/ws/src
RUN mv /opt/carla-ros-bridge/src/ros_compatibility /opt/ws/src/ros_compatibility
RUN rm -rf /opt/carla-ros-bridge
RUN apt update && apt install -y \
    git \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-pointcloud-to-laserscan \
    python3-pip

RUN pip install transforms3d

    
WORKDIR /opt/ws

RUN git clone https://github.com/carla-simulator/ros-carla-msgs.git src/ros-carla-msgs
RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    if [ "$ROS_VERSION" == "2" ]; then colcon build; else catkin_make install; fi'

COPY interfaces src/

RUN /bin/bash -c 'source /opt/ros/$ROS_DISTRO/setup.bash; \
    if [ "$ROS_VERSION" == "2" ]; then source install/setup.bash && colcon build; else source devel/setup.bash && catkin_make install; fi'

RUN echo "source /opt/ws/install/setup.bash" >> /root/.bashrc
RUN echo "source /opt/carla/setup.bash" >> /root/.bashrc

COPY ./.docker_utils/ros_entrypoint.sh /
COPY ./.docker_utils/parse_ros2_params.py /
