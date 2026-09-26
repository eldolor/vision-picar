# The ROS 2 container -- the ONLY place ROS exists in this project
# (tests/test_ros_containment.py). Humble, because JetPack 6 is Ubuntu 22.04
# and the car will run the same distribution this is tested against.
#
#   docker build -t vision-picar-ros service/slam
#
# Everything runs in ONE container so DDS stays container-internal
# (PLAN-ros-alignment.md section 4): no multicast across Docker's network.
# It reaches robot/server.py over HTTP at ROBOT_URL, and publishes the
# bridge's HTTP port back out.
FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-xacro \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-ros2-control \
        ros-humble-ros2-controllers \
        ros-humble-twist-mux \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        liburdfdom-tools \
        libcurl4-openssl-dev \
        nlohmann-json3-dev \
        python3-requests \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src /ws/src
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
