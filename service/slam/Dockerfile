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
        ros-humble-slam-toolbox \
        ros-humble-rmw-cyclonedds-cpp \
        ros-humble-navigation2 \
        ros-humble-nav2-collision-monitor \
        ros-humble-nav2-regulated-pure-pursuit-controller \
        ros-humble-tf2-ros \
        ros-humble-tf2-tools \
        liburdfdom-tools \
        libcurl4-openssl-dev \
        nlohmann-json3-dev \
        python3-requests \
    && rm -rf /var/lib/apt/lists/*

# slam_toolbox from source, pinned. The apt release (2.6.10) predates the
# branch's `restamp_tf` although both say 2.6.10, and without it a robot AT
# REST publishes an ever-older map -> odom: nav2's controller refuses it, the
# robot never moves, and no scan is processed again (R6, 3.15). Built in its
# own underlay so the project's packages stay a separate, fast layer.
ARG SLAM_TOOLBOX_SHA=1729c0fe5832c796030bf183d0013f7eed594ae7
RUN apt-get update && apt-get install -y --no-install-recommends git \
        libceres-dev liblapack-dev libblas-dev libsuitesparse-dev libtbb-dev \
        ros-humble-interactive-markers ros-humble-pluginlib ros-humble-rviz-common \
        ros-humble-rviz-default-plugins ros-humble-rviz2 qtbase5-dev \
        ros-humble-message-filters ros-humble-nav2-map-server \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /underlay/src && cd /underlay/src \
    && git clone https://github.com/SteveMacenski/slam_toolbox.git \
    && cd slam_toolbox && git checkout ${SLAM_TOOLBOX_SHA} \
    && cd /underlay && . /opt/ros/humble/setup.sh \
    && colcon build --packages-select slam_toolbox --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf /underlay/build /underlay/log

WORKDIR /ws
COPY src /ws/src
RUN . /opt/ros/humble/setup.sh && . /underlay/install/setup.sh && colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

# Cyclone DDS, not Humble's default Fast DDS -- the middleware Nav2's own
# documentation recommends for Humble. R6's first runs lost map -> odom
# delivery from slam_toolbox to nav2's controller 20 s after start, while
# odom -> base_footprint on the same /tf topic kept arriving and `ros2 topic
# echo` saw both: the controller then planned from a pose minutes old and
# declared every goal reached in 0.08 s.
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
