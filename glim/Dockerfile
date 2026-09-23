FROM koide3/glim_ros2:jazzy

# Pridanie CycloneDDS, aby GLIM vedel komunikovať s MowgliNext stackom,
# ktorý používa rmw_cyclonedds_cpp namiesto defaultného FastDDS.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
