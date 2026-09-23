# ---------------------------------------------------------------------------
# lidarloc_dds
#
# Image pre rsasaki0109/lidar_localization_ros2 (+ jeho závislosť ndt_omp_ros2).
# Postavené na rovnakom ROS 2 distro (kilted) ako velodyne_driver/glim images,
# aby DDS komunikácia (cyclonedds) medzi kontajnermi fungovala bez problémov.
#
# Build (z rootu repozitára):
#   docker compose build lidar_localization
# alebo priamo:
#   docker build -t lidarloc_dds:latest ./lidar_localization
# ---------------------------------------------------------------------------

FROM ros:kilted-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kilted
ENV LANG=C.UTF-8

# --- systémové závislosti na build ------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        wget \
        build-essential \
        cmake \
        python3-pip \
        python3-colcon-common-extensions \
        python3-vcstool \
        python3-rosdep \
        libpcl-dev \
        libeigen3-dev \
        libomp-dev \
        ros-${ROS_DISTRO}-pcl-conversions \
        ros-${ROS_DISTRO}-pcl-ros \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-eigen \
        ros-${ROS_DISTRO}-nav2-common \
        ros-${ROS_DISTRO}-cyclonedds \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

# --- workspace ---------------------------------------------------------------
WORKDIR /opt/lidarloc_ws/src

RUN git clone --depth 1 https://github.com/rsasaki0109/lidar_localization_ros2.git

RUN if [ -f lidar_localization_ros2/dependencies.repos ]; then \
        vcs import . < lidar_localization_ros2/dependencies.repos; \
    else \
        git clone --depth 1 https://github.com/rsasaki0109/ndt_omp_ros2.git; \
    fi

WORKDIR /opt/lidarloc_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "ndt_omp_ros2" || true

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# --- runtime konfigurácia ----------------------------------------------------
RUN mkdir -p /opt/lidarloc_ws/param /maps

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /opt/lidarloc_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/lidarloc_ws/install/setup.bash && exec \"$@\"", "--"]

CMD ["ros2", "launch", "lidar_localization_ros2", "lidar_localization.launch.py"]
