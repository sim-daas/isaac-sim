FROM nvcr.io/nvidia/isaac-sim:5.1.0

USER root
# Create group with matching GID and add isaac-sim user
RUN groupadd -g ${OVSHARE_GID} ovshare \
    && usermod -aG ovshare isaac-sim

ENV ROS_DISTRO=jazzy \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    LD_LIBRARY_PATH=/isaac-sim/exts/omni.isaac.ros2_bridge/jazzy/lib

# Ensure group-writable default behavior
RUN sed -i 's/^UMASK.*/UMASK 002/' /etc/login.defs || true
