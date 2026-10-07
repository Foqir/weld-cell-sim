# Digital twin of a robotic arc-welding cell: HD Hyundai Robotics robot (official ROS 2 packages)
# + MoveIt 2 + ros2_control + Gazebo Harmonic (ROS 2 Jazzy vendor packages) in one image.
# Independent of the px4-vtol-sim project: own base image, own tags, own backups.
# Stages (deps -> hdr-build -> final) are built and checkpointed one by one by build.ps1.
FROM ros:jazzy-ros-base AS deps

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_WS=/opt/ros2_ws \
    QT_QPA_PLATFORM=xcb

# apt retries + a persistent .deb cache so a dropped download doesn't restart everything.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean \
    && printf 'Acquire::Retries "10";\nAcquire::http::Timeout "60";\n' > /etc/apt/apt.conf.d/80-retries \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        git nano less wget curl build-essential \
        python3-colcon-common-extensions python3-numpy python3-matplotlib python3-yaml \
        libboost-system-dev libcurl4-openssl-dev liburdfdom-tools nlohmann-json3-dev \
        libgl1-mesa-dri mesa-utils \
        ros-jazzy-moveit \
        ros-jazzy-moveit-visual-tools \
        ros-jazzy-pilz-industrial-motion-planner \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros-gz \
        ros-jazzy-rviz2 \
        ros-jazzy-xacro \
        ros-jazzy-robot-state-publisher \
        ros-jazzy-joint-state-publisher \
        ros-jazzy-joint-state-publisher-gui \
        ros-jazzy-tf2-ros \
        ros-jazzy-srdfdom \
        ros-jazzy-launch-testing-ament-cmake \
        ros-jazzy-launch-testing-ros \
        ros-jazzy-ament-cmake-pytest \
        ros-jazzy-ament-lint-auto \
        ros-jazzy-ament-lint-common \
    && rm -rf /var/lib/apt/lists/*

# Official HD Hyundai Robotics packages (cloned into ros2_ws/src on the host, branch jazzy).
FROM deps AS hdr-build
COPY ros2_ws/src/hdr_description   ${ROS_WS}/src/hdr_description
COPY ros2_ws/src/hdr_client_driver ${ROS_WS}/src/hdr_client_driver
COPY ros2_ws/src/hdr_ros2_driver   ${ROS_WS}/src/hdr_ros2_driver
COPY ros2_ws/src/hdr_simulation_gz ${ROS_WS}/src/hdr_simulation_gz
RUN . /opt/ros/jazzy/setup.sh \
    && cd ${ROS_WS} \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Our welding cell package (world, torch, part, seam programs, demo) - rebuilds in seconds.
FROM hdr-build AS final
COPY ros2_ws/src/weld_cell ${ROS_WS}/src/weld_cell
RUN . /opt/ros/jazzy/setup.sh \
    && cd ${ROS_WS} \
    && colcon build --packages-select weld_cell

COPY scripts/ /opt/scripts/
RUN sed -i 's/\r$//' /opt/scripts/*.sh && chmod +x /opt/scripts/*.sh \
    && echo 'source /opt/scripts/env.sh' >> /root/.bashrc

WORKDIR /root
CMD ["/opt/scripts/start_all.sh"]
