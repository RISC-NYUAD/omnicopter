FROM osrf/ros:jazzy-desktop AS base

ENV DEBIAN_FRONTEND=noninteractive

RUN echo "deb [signed-by=/etc/apt/keyrings/xpra-keyring.gpg] https://xpra.org/ noble main" > /etc/apt/sources.list.d/xpra.list \
    && curl -fsSL https://xpra.org/gpg.asc | gpg --dearmor -o /etc/apt/keyrings/xpra-keyring.gpg >/dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/apt-fast.gpg] http://ppa.launchpad.net/apt-fast/stable/ubuntu noble main" > /etc/apt/sources.list.d/apt-fast.list \
    && curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xBC5934FD3DEBD4DAEA544F791E2824A7F22B44BD" | gpg --dearmor -o /etc/apt/keyrings/apt-fast.gpg >/dev/null

RUN apt-get update

RUN DEBIAN_FRONTEND=noninteractive apt-get install -y apt-fast \
    && apt-get clean

# Base dependencies
RUN sudo apt-get update
RUN sudo apt-get install -y --no-install-recommends \
    libserial-dev \
    libeigen3-dev \
    netbase \
    iputils-ping \
    setserial \
    libyaml-cpp-dev \
    libpcap-dev \
    libboost-all-dev \
    libpcl-dev \
    libreadline-dev \
    menu \
    menu-xdg \
    nano \
    && apt-get clean

# Additional dependencies for Gazebo
RUN sudo apt-get install -y curl lsb-release gnupg \
    && sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && sudo apt-get install -y gz-harmonic \
    && apt-get clean

RUN mkdir -p /ros_ws/src
WORKDIR /ros_ws

# Copy your ROS2 workspace into the container
COPY ./src /ros_ws/src
COPY ./bash_Scripts /ros_ws/bash_Scripts

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build
RUN echo "source /ros_ws/install/setup.bash" >> /root/.bashrc

RUN DEBIAN_FRONTEND=noninteractive apt-fast install -y \
    xpra-server \
    && apt-get clean

# Set the entrypoint and ensure ROS environment is sourced
CMD ["/bin/bash", "-c", "xpra start --no-daemon --bind-ws=0.0.0.0:8080 --webcam=no --pulseaudio=no --mdns=no"]

