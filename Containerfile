FROM docker.io/px4io/px4-dev-ros2-gazebo@sha256:4e7bb18e46f8007d3e601e25597f84117ef14d282df10999e2dc0a9fe7d4e1f2

LABEL org.opencontainers.image.source="https://github.com/MYahsap/Bird-Safe-Window-Marker-Instalation-Drone"
LABEL org.opencontainers.image.description="PX4 v1.17, ROS 2 Jazzy, and Gazebo environment for the Holybro X650 simulation"

USER root
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        qtwayland5 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace
CMD ["bash"]
