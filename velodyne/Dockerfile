# ---------------------------------------------------------------------------
# vlp16_dds
#
# TODO: Toto je placeholder. Doplň sem presné kroky, ktorými si postavil
# existujúci "vlp16_dds:latest" image (ten, čo je referencovaný v pôvodnom
# docker-compose.yaml). Typicky pôjde o:
#
#   FROM ros:kilted-ros-base
#   RUN apt-get update && apt-get install -y --no-install-recommends \
#         ros-kilted-velodyne \
#         ros-kilted-velodyne-driver \
#         ros-kilted-velodyne-pointcloud \
#         ros-kilted-velodyne-laserscan \
#         ros-kilted-cyclonedds \
#         ros-kilted-rmw-cyclonedds-cpp \
#       && rm -rf /var/lib/apt/lists/*
#
# Ak namiesto apt balíkov staviaš velodyne driver zo zdroja (napr. kvôli
# špecifickej vetve alebo patchu), sem príde workspace + colcon build,
# rovnako ako v lidar_localization/Dockerfile.
#
# Až doplníš skutočný obsah, zmaž tento komentár.
# ---------------------------------------------------------------------------

FROM ros:kilted-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kilted

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-velodyne \
        ros-${ROS_DISTRO}-velodyne-driver \
        ros-${ROS_DISTRO}-velodyne-pointcloud \
        ros-${ROS_DISTRO}-velodyne-laserscan \
        ros-${ROS_DISTRO}-cyclonedds \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "launch", "velodyne", "velodyne-all-nodes-VLP16-launch.py"]
