# syntax=docker/dockerfile:1.6
# ---------------------------------------------------------------------------
# OHM (CSIRO) + ROS 2 Humble + CUDA — live mapping + offline tools image.
#
# CUDA 12.2 / Ubuntu 22.04 (Jammy). Jammy is the native Ubuntu for ROS 2
# Humble, which matches the docker-stereo image in this workspace.
#
# Layout:
#   /opt/ohm                    OHM source tree (pinned via OHM_REF)
#   /usr/local/{lib,include}    OHM installed here by cmake --install
#   /root/user_ws               bind-mounted ament workspace (ohm_ros2 package)
#
# The ohm_ros2 package is NOT built into the image — it's mounted from the
# host under /root/user_ws so you can iterate on the node without rebuilding.
# Run `colcon build --symlink-install` inside the container on first run.
# ---------------------------------------------------------------------------
FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive
ARG ROS_DISTRO=humble
ARG OHM_REPO=https://github.com/csiro-robotics/ohm.git
ARG OHM_REF=master
ARG BUILD_JOBS=16

ENV ROS_DISTRO=${ROS_DISTRO} \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    TZ=Etc/UTC \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
    NVIDIA_VISIBLE_DEVICES=all

# ---------------------------------------------------------------------------
# 1. Core build tooling + locale
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        gnupg2 \
        locales \
        lsb-release \
        ninja-build \
        pkg-config \
        python3 \
        python3-pip \
        software-properties-common \
        sudo \
        wget \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 2. OHM required + recommended + heightmap dependencies
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        zlib1g-dev \
        libglm-dev \
        libgtest-dev \
        googletest \
        libtbb-dev \
        libpdal-dev \
        doxygen \
        graphviz \
        libeigen3-dev \
        libglew-dev \
        libglfw3-dev \
        libpng-dev \
        libgl1-mesa-dev \
        libglu1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 3. ROS 2 Humble apt source (jammy) + desktop install
# ---------------------------------------------------------------------------
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb \
        "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && apt-get update && apt-get install -y /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-desktop \
        ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-tf2-ros \
        ros-${ROS_DISTRO}-tf2-sensor-msgs \
        ros-${ROS_DISTRO}-message-filters \
        ros-dev-tools \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-argcomplete \
    && rosdep init \
    && rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------------------------
# 4. Clone OHM at the pinned ref.
# ---------------------------------------------------------------------------
WORKDIR /opt
RUN git clone ${OHM_REPO} ohm \
    && cd ohm \
    && git checkout ${OHM_REF} \
    && git submodule update --init --recursive

# ---------------------------------------------------------------------------
# 4b. PDAL stream reader patch.
#     Without this, SlamCloudLoader treats Colour|Intensity|ReturnNumber as
#     required and no standard LAZ file passes validation. Matters when using
#     slamio / ohmpop with LAZ input.
# ---------------------------------------------------------------------------
RUN sed -i \
        's@(desired_channels_ == DataChannel::None) ? (DataChannel::Position | DataChannel::Time) : desired_channels_@DataChannel::Position | DataChannel::Time@' \
        /opt/ohm/slamio/PointCloudReaderPdal.cpp \
    && ! grep -q 'desired_channels_ == DataChannel::None) ? (DataChannel::Position' \
        /opt/ohm/slamio/PointCloudReaderPdal.cpp \
    && echo "OHM PDAL desired-as-required patch applied."

# ---------------------------------------------------------------------------
# 5. Configure + build + install OHM (CUDA backend).
# ---------------------------------------------------------------------------
WORKDIR /opt/ohm
RUN cmake -S . -B build -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DOHM_BUILD_SHARED=ON \
        -DOHM_FEATURE_OPENCL=OFF \
        -DOHM_FEATURE_CUDA=ON \
        -DOHM_FEATURE_HEIGHTMAP=ON \
        -DOHM_FEATURE_HEIGHTMAP_IMAGE=ON \
        -DOHM_FEATURE_PDAL=ON \
        -DOHM_FEATURE_THREADS=ON \
        -DOHM_FEATURE_EIGEN=ON \
        -DOHM_FEATURE_TEST=OFF \
        -DOHM_EMBED_GPU_CODE=ON \
        -DOHM_TES_DEBUG=OFF \
        -DOHM_BUILD_DOXYGEN=OFF \
    && cmake --build build --parallel ${BUILD_JOBS} \
    && cmake --install build \
    && ldconfig \
    && rm -rf /opt/ohm/build

# ---------------------------------------------------------------------------
# 6. Bootstrap — source ROS 2 + the user workspace in every interactive shell.
#    The user_ws mount is expected at /root/user_ws; the `[ -f ... ]` guard
#    keeps the shell happy on a fresh checkout before the first colcon build.
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo 'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}' >> /root/.bashrc \
    && echo '[ -f /root/user_ws/install/setup.bash ] && source /root/user_ws/install/setup.bash' >> /root/.bashrc

ENV PATH=/usr/local/bin:${PATH} \
    LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}

# ---------------------------------------------------------------------------
# 7. Supervisor for the UI portal.
# ---------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends supervisor \
    && rm -rf /var/lib/apt/lists*

# Data volume for bags / saved .ohm files.
RUN mkdir -p /data /var/log/supervisor
VOLUME ["/data"]

COPY entrypoint.sh /entrypoint.sh
COPY supervisord.conf /etc/supervisor/supervisord.conf
RUN chmod +x /entrypoint.sh

WORKDIR /root/user_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/usr/bin/supervisord", "-c", "/etc/supervisor/supervisord.conf"]
