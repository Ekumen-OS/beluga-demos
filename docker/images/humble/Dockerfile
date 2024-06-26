FROM ros:humble-ros-base-jammy AS cacher

WORKDIR /ws/src

COPY . beluga_demo/

RUN mkdir -p /tmp/ws/src \
  && find ./ -name "package.xml" | xargs cp --parents -t /tmp/ws/src \
  && find ./ -name "COLCON_IGNORE" | xargs cp --parents -t /tmp/ws/src \
  || true

FROM ros:humble-ros-base-jammy AS gazebo_model_downloader

RUN sudo apt-get update \
  && sudo apt-get install -y \
     wget \
  && sudo rm -rf /var/lib/apt/lists/*

RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
  && wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
  && sudo apt-get update \
  && sudo apt-get install -y \
     libgz-fuel-tools9-dev \
  && sudo rm -rf /var/lib/apt/lists/*

COPY docker/files/fuel_models.txt /tmp/fuel_models.txt
RUN cat /tmp/fuel_models.txt | xargs -I {} gz fuel download -u https://fuel.gazebosim.org/1.0/OpenRobotics/models/{}
RUN cd ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models \
  && while read f; do for u in `ls "$f"`; do cd "$f"/"$u"; mv * ..; cd .. ; rm -rf $u; cd .. ; done; done < /tmp/fuel_models.txt \
  && mv dining%20table dining_table \
  && mv office%20chair office_chair  \
  && mv office%20desk  office_desk

RUN cd ~/.gz/fuel/fuel.gazebosim.org/openrobotics/models \
  && tar cvfz fuel_models.tar.gz * \
  && mv fuel_models.tar.gz /tmp/fuel_models.tar.gz

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

# Fix to gazebo crashing when loading simulation. It's unclear why this fixes it,
# and why it happens (the same repo version a couple of months ago didn't have this issue,
# so the change is in gazebo or ros). See
# https://answers.ros.org/question/358847/cannot-launch-gzclient-on-a-launch-file-results-in-shared_ptr-assertion-error/
RUN bash -c "echo '. /usr/share/gazebo/setup.sh' >> /home/$USER/.bashrc"

RUN bash -c "echo 'ros2 daemon start' >> /home/$USER/.bashrc"

ENV WITHIN_DEV 1
ENV TURTLEBOT3_MODEL waffle_pi

ENV MAKEFLAGS "-j 2"

RUN mkdir -p ~/.gazebo/models \
  mkdir -p ~/repos \
  && cd ~/repos \
  && git clone --depth=1 --recursive https://github.com/koide3/gazebo_apriltag.git \
  && git clone --depth=1 --recursive https://github.com/osrf/gazebo_models.git \
  && mv gazebo_apriltag/models/* ~/.gazebo/models/ \
  && mv gazebo_models/* ~/.gazebo/models/ \
  && rm -rf gazebo_apriltag gazebo_models

COPY --from=gazebo_model_downloader --chown=$USER:$GROUP /tmp/fuel_models.tar.gz /home/developer/.gazebo/models/
RUN cd ~/.gazebo/models \
  && tar xvfz fuel_models.tar.gz \
  && rm fuel_models.tar.gz

ENV SHELL /bin/bash
ENTRYPOINT ["fixuid", "-q", "/ros_entrypoint.sh", "/bin/bash"]
