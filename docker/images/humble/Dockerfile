FROM ros:humble-ros-base-jammy AS package_cacher

WORKDIR /ws/src

COPY . beluga_demo/

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:humble-ros-base-jammy AS gz_model_cacher

WORKDIR /ws/src

COPY . beluga_demo/
RUN mkdir -p /tmp/ws/src \
  && find ./ -name "*.world" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "*.sdf" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:humble-ros-base-jammy AS openvdb-builder

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    git \
    libboost-all-dev \
    libtbb-dev \
    libblosc-dev \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /opt

RUN git clone -b v8.2.0 https://github.com/AcademySoftwareFoundation/openvdb.git \
  && cd openvdb \
  && mkdir build && cd build \
  && cmake .. \
  && make -j$(nproc)

FROM ros:humble-ros-base-jammy AS builder

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    gdb \
    gnupg \
    git \
    python3-pip \
    python3-venv \
    mc \
    lsb-release \
    tmux \
  && rm -rf /var/lib/apt/lists/*

ARG GZ_VERSION="harmonic"
RUN curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list \
  && apt-get update && apt-get install --no-install-recommends -y $(if [[ "${GZ_VERSION}" > "fortress" ]]; then echo gz; else echo ignition; fi)-${GZ_VERSION}

RUN pip install pre-commit==2.20.0

ARG USER=developer
ARG GROUP=ekumen

RUN addgroup --gid 1010 $GROUP \
  && adduser --uid 1010 --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
  && adduser $USER sudo \
  && adduser $USER dialout \
  && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

COPY docker/files/fixuid_config.yml /etc/fixuid/config.yml
RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - \
  && chmod 4755 /usr/local/bin/fixuid \
  && cd /etc/fixuid \
  && sed -i "s/_USER_/$USER/" config.yml \
  && sed -i "s/_GROUP_/$GROUP/" config.yml

USER $USER:$GROUP

ENV USER_WS=/home/$USER/ws
RUN mkdir -p /$USER_WS

WORKDIR /tmp

WORKDIR $USER_WS

RUN colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml \
  && colcon mixin update default
COPY --chown=$USER:$GROUP docker/files/colcon_defaults.yaml /home/$USER/.colcon/defaults.yaml
RUN mkdir -p /home/$USER/.ccache $USER_WS/src

#
# Install base system external dependencies

COPY docker/files/humble_base.repos /tmp/humble_base.repos
RUN cd src/ \
  && mkdir -p external-deps \
  && cd external-deps \
  && vcs import < /tmp/humble_base.repos

#
# Apply patches

COPY docker/files/patches/apriltag_ros.patch /tmp/apriltag_ros.patch
RUN cd src/external-deps/apriltag_ros \
  && cat /tmp/apriltag_ros.patch | git apply -p1

COPY docker/files/patches/nav2_minimal_turtlebot_simulation.patch /tmp/nav2_minimal_turtlebot_simulation.patch
RUN cd src/external-deps/nav2_minimal_turtlebot_simulation \
  && cat /tmp/nav2_minimal_turtlebot_simulation.patch | git apply -p1

#
# Install project dependencies

COPY --from=package_cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/
RUN sudo apt-get update \
  && . /opt/ros/humble/setup.sh \
  && rosdep update \
  && rosdep install -i -y --from-path src \
  && sudo rm -rf /var/lib/apt/lists/*

#
# Install aliases and setup environment

COPY --chown=$USER:$GROUP docker/files/DOTaliases /home/$USER/.bash_aliases

RUN bash -c "echo '. /opt/ros/humble/setup.bash' >> /home/$USER/.bashrc"

RUN bash -c "echo 'ros2 daemon start' >> /home/$USER/.bashrc"

ENV WITHIN_DEV=1
ENV MAKEFLAGS="-j 2"

#
# Install source distributed Gazebo models
COPY docker/files/patches/gazebo_apriltag.patch /tmp/gazebo_apriltag.patch
RUN mkdir -p ~/.gazebo/models \
  && git clone --depth=1 --recursive https://github.com/rickarmstrong/gazebo_apriltag.git \
  && cd gazebo_apriltag \
  && (cat /tmp/gazebo_apriltag.patch | git apply -p1) \
  && python3 -m venv venv \
  && source venv/bin/activate \
  && pip install -r requirements.txt \
  && python3 generate.py \
  && mv models/* ~/.gazebo/models/. \
  && cd - \
  && rm -rf gazebo_apriltag

#
# Cache Gazebo Fuel models
COPY --from=gz_model_cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/
RUN find src -name '*.world' -or -name '*.sdf' | \
    grep -Roh -e 'https://fuel.gazebosim.org[^<]*' src | \
    sort | uniq | xargs -I{} gz fuel download -u {}

ENV GZ_SIM_RESOURCE_PATH=/home/$USER/.gazebo/models

#
# Install OpenVDB files
COPY --from=openvdb-builder --chown=$USER:$GROUP /opt/openvdb/ /opt/openvdb
RUN cd /opt/openvdb/build \
    && sudo make install \
    && sudo rm -rf /opt/openvdb

ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
