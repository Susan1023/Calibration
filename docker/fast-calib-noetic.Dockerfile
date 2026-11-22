FROM ros:noetic-ros-base-focal

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get update -o Acquire::Retries=5 && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y -o Acquire::Retries=5 --no-install-recommends \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-cv-bridge \
    ros-noetic-image-geometry \
    ros-noetic-message-filters \
    ros-noetic-tf \
    libopencv-dev \
    libopencv-contrib-dev \
    libpcl-dev

WORKDIR /calib_ws
COPY src/FAST-Calib /calib_ws/src/fast_calib
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash && catkin_make -j1 -DCMAKE_BUILD_TYPE=Release'
