FROM ros:lyrical

WORKDIR /workspace

# Install cv_bridge and OpenCV dependencies
RUN apt-get update && apt-get install -y \
    ros-lyrical-cv-bridge \
    python3-opencv \
    libopencv-dev \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY docker/ros-entrypoint.sh /usr/local/bin/ros-entrypoint
RUN chmod +x /usr/local/bin/ros-entrypoint

ENTRYPOINT ["ros-entrypoint"]
CMD ["bash"]
