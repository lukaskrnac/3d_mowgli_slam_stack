# ---------------------------------------------------------------------------
# lidarloc_dds
#
# Image pre rsasaki0109/lidar_localization_ros2 (+ jeho závislosť ndt_omp_ros2).
#
# Upstream je pinnutý na konkrétny commit a patchnutý (patches/*.patch):
#   0001 - nový parameter `publish_tf` (default true = upstream správanie).
#          S `publish_tf: false` node počíta a publikuje /pcl_pose, ale
#          NEBROADCASTUJE žiadny TF (map->base_link ani map->odom). TF strom
#          vlastní výhradne MowgliNext fusion_graph.
# Pri zmene LIDARLOC_REF over, že patche stále idú aplikovať (build inak padne).
#
# Build (z rootu repozitára):
#   docker compose build lidar_localization
# alebo priamo:
#   docker build -t lidarloc_dds:latest ./lidar_localization
# ---------------------------------------------------------------------------

FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV LANG=C.UTF-8

ARG LIDARLOC_REF=537ac5b02155d78e2ce519b8241fe39c6574a936
ARG NDT_OMP_REF=8b77fa5a6cdcad45bf35918361c892b6d94a287e

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

COPY patches/ /opt/lidarloc_patches/

RUN git init -q lidar_localization_ros2 && cd lidar_localization_ros2 && \
    git remote add origin https://github.com/rsasaki0109/lidar_localization_ros2.git && \
    git fetch -q --depth 1 origin ${LIDARLOC_REF} && git checkout -q FETCH_HEAD && \
    for p in /opt/lidarloc_patches/*.patch; do echo "applying $p" && git apply "$p" || exit 1; done

RUN git init -q ndt_omp_ros2 && cd ndt_omp_ros2 && \
    git remote add origin https://github.com/rsasaki0109/ndt_omp_ros2.git && \
    git fetch -q --depth 1 origin ${NDT_OMP_REF} && git checkout -q FETCH_HEAD

WORKDIR /opt/lidarloc_ws

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y --skip-keys "ndt_omp_ros2" || true; \
    rm -rf /var/lib/apt/lists/*

# Jeden cc1plus na lidar_localization_ros2 zoberie ~7 GB RAM -> pri plnom paralelizme
# (16 GB GitHub runner / kosačka) build náhodne padá na OOM. Uprav cez --build-arg.
ARG BUILD_JOBS=2
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    MAKEFLAGS="-j${BUILD_JOBS}" colcon build --symlink-install --parallel-workers 1 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# --- runtime konfigurácia ----------------------------------------------------
RUN mkdir -p /opt/lidarloc_ws/param /maps /config

# Default configy (v compose sa prekrývajú mountom z repa, aby sa dali meniť bez rebuildu)
COPY config/ /config/
COPY tools/ /opt/tools/

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /opt/lidarloc_ws/install/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/lidarloc_ws/install/setup.bash && exec \"$@\"", "--"]

CMD ["ros2", "launch", "lidar_localization_ros2", "lidar_localization.launch.py", "localization_param_dir:=/config/lidar_localization.yaml"]
