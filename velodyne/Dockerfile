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
    
    # Minimálna vzdialenosť bodov: 0.2 m
RUN sed -i 's/min_range:.*/min_range: 0.2/' \
    /opt/ros/kilted/share/velodyne_pointcloud/config/VLP16-velodyne_transform_node-params.yaml

# Build sa robí len raz (docker compose build). Výsledný image by mal
# vyjsť rádovo ~500 MB - 1 GB, nie niekoľko GB.
