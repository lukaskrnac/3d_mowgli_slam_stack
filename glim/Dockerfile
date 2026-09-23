# ---------------------------------------------------------------------------
# glim_dds
#
# TODO: Toto je placeholder. Doplň sem presné kroky, ktorými si postavil
# existujúci "glim_dds:latest" image (bez CUDA verzia spomínaná v pôvodnom
# docker-compose.yaml). GLIM sa typicky stavia zo zdroja (git clone + build),
# keďže nemá apt balík. Základná kostra:
#
#   FROM ros:kilted-ros-base
#   RUN apt-get update && apt-get install -y --no-install-recommends \
#         build-essential cmake git \
#         libeigen3-dev libboost-all-dev libomp-dev \
#         libopencv-dev libpcl-dev \
#         ros-kilted-cyclonedds ros-kilted-rmw-cyclonedds-cpp \
#       && rm -rf /var/lib/apt/lists/*
#
#   WORKDIR /glim_ws/src
#   RUN git clone --recursive https://github.com/koide3/glim.git
#   RUN git clone --recursive https://github.com/koide3/glim_ros2.git   # alebo aktuálny názov ROS2 wrapperu
#   WORKDIR /glim_ws
#   RUN colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
#
# Doplň presné repo/tagy, ktoré aktuálne používaš (najmä ak máš vlastný fork
# alebo patch pre RelocalizationModule / config_relocalization.json).
#
# Až doplníš skutočný obsah, zmaž tento komentár.
# ---------------------------------------------------------------------------

FROM ros:kilted-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=kilted

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        libeigen3-dev \
        libboost-all-dev \
        libomp-dev \
        libopencv-dev \
        libpcl-dev \
        mesa-utils \
        libgl1-mesa-glx \
        ros-${ROS_DISTRO}-cyclonedds \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# --- TODO: sem doplň git clone + colcon build pre glim/glim_ros -----------

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec \"$@\"", "--"]
CMD ["ros2", "run", "glim_ros", "glim_rosnode"]
