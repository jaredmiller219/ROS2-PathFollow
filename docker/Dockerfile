FROM ros:lyrical

WORKDIR /workspace

COPY docker/ros-entrypoint.sh /usr/local/bin/ros-entrypoint
RUN chmod +x /usr/local/bin/ros-entrypoint

ENTRYPOINT ["ros-entrypoint"]
CMD ["bash"]
