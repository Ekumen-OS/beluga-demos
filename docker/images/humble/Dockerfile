FROM ros:humble-ros-base-jammy AS cacher

WORKDIR /ws/src

COPY . beluga_demo/

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:humble-ros-base-jammy AS builder

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update \
  && apt-get install --no-install-recommends -y \
    ccache \
    curl \
    gdb \
    git \
    python3-pip \
    mc \
    tmux \
  && rm -rf /var/lib/apt/lists/*

RUN pip install \
  pre-commit==2.20.0

ARG USER=developer
ARG GROUP=ekumen

RUN addgroup --gid 1000 $GROUP \
  && adduser --uid 1000 --ingroup $GROUP --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER \
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

ENV USER_WS /home/$USER/ws
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

RUN sudo apt-get update \
  && . /opt/ros/humble/setup.sh \
  && rosdep update \
  && rosdep install -i -y --from-path src \
  && sudo rm -rf /var/lib/apt/lists/*

#
# Apply patches

COPY docker/files/patches/apriltag_ros.patch /tmp/apriltag_ros.patch
RUN cd src/external-deps/apriltag_ros \
  && cat /tmp/apriltag_ros.patch | git apply -p1

COPY docker/files/patches/turtlebot3_gazebo.patch /tmp/turtlebot3_gazebo.patch
RUN cd src/external-deps/turtlebot3_simulations \
  && cat /tmp/turtlebot3_gazebo.patch | git apply -p1

#
# Install project dependencies

COPY --from=cacher --chown=$USER:$GROUP /tmp/ws/ $USER_WS/
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

ENV WITHIN_DEV 1
ENV TURTLEBOT3_MODEL waffle_pi

ENV MAKEFLAGS "-j 2"

ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
