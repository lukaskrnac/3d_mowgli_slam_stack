FROM ros:kilted-ros-core

# ros-core neobsahuje colcon/rosdep ani iné dev nástroje, takže je výrazne
# menší ako ros-base. Na samotné spustenie hotového driver-balíka to stačí.
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-kilted-velodyne \
        ros-kilted-velodyne-driver \
        ros-kilted-velodyne-pointcloud \
        ros-kilted-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Build sa robí len raz (docker compose build). Výsledný image by mal
# vyjsť rádovo ~500 MB - 1 GB, nie niekoľko GB.
