FROM nvcr.io/nvidia/isaac-sim:5.1.0

# build args passed from docker build
ARG OVSHARE_GID=30001

USER root

# Create shared group and add isaac-sim user to it
RUN groupadd -g ${OVSHARE_GID} ovshare \
    && usermod -aG ovshare isaac-sim

# Make group-writable the default
RUN sed -i 's/^UMASK.*/UMASK 002/' /etc/login.defs || true

ENV ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    LD_LIBRARY_PATH=/isaac-sim/exts/isaacsim.ros2_bridge/jazzy/lib:/isaac-sim/exts/isaacsim.ros2.bridge/jazzy/lib

# Drop back to non-root (important)
USER isaac-sim
