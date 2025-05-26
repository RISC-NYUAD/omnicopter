FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y curl gnupg2 lsb-release

# Add Xpra and apt-fast repositories and keys
RUN mkdir -p /etc/apt/keyrings && \
    echo "deb [signed-by=/etc/apt/keyrings/xpra-keyring.gpg] https://xpra.org/ focal main" > /etc/apt/sources.list.d/xpra.list && \
    curl -fsSL https://xpra.org/gpg.asc | gpg --dearmor -o /etc/apt/keyrings/xpra-keyring.gpg >/dev/null


ENV USERNAME=user
ARG user_id=2000  # Default value, can be overridden by docker-compose.yml

# Base dependencies installation (including i2c-tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libserial-dev \
    libeigen3-dev \
    ros-noetic-tf \
    ros-noetic-serial \
    git \
    libudev-dev \
    netbase \
    libgpiod-dev \
    gpiod \
    ros-noetic-vrpn \
    ros-noetic-vrpn-client-ros \
    iputils-ping \
    setserial \
    libreadline-dev \
    nano \
    i2c-tools \
    lsb-release \
    wget \
    gnupg2 \
    libssl-dev \
    libusb-1.0-0-dev \
    libusb-1.0-0 \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    guvcview \
    python3 \
    python3-dev \
    python3-pip \
    udev \
    apt-transport-https \
    ca-certificates \
    curl \
    software-properties-common \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies (NumPy >= 1.20, Matplotlib)
RUN python3 -m pip install --upgrade pip \
    && python3 -m pip install numpy>=1.20 matplotlib>=3.0

# Switch to a non-root user and add them to necessary groups
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && adduser $USERNAME sudo \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME \
    && adduser $USERNAME dialout \
    && adduser $USERNAME i2c  # Add the user to the i2c group

# Clone and build librealsense v2.50.0
RUN git clone https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    git checkout v2.50.0 && \
    mkdir build && cd build && \
    cmake .. -DBUILD_EXAMPLES=false && \
    make -j$(nproc) && \
    sudo make install && \
    echo "/usr/local/lib" | tee /etc/ld.so.conf.d/librealsense.conf && \
    ldconfig && \
    rm -rf /tmp/librealsense

# Install ROS RealSense wrapper from apt (matches librealsense v2.50.0)
RUN apt-get update && \
    apt-get install -y ros-${ROS_DISTRO}-realsense2-camera && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Xpra server
RUN apt-get update && \
    apt-get install -y xpra xpra-html5 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Copy your ROS1 workspace into the container
COPY ./flightSoftware /opt/ros/noetic/flightSoftware/src

USER $USERNAME
RUN pip install git+https://github.com/hungpham2511/toppra

# Update .bashrc to source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /opt/ros/noetic/flightSoftware/devel/setup.bash ]; then source /opt/ros/noetic/flightSoftware/devel/setup.bash; fi" >> ~/.bashrc

# Entry point to ensure ROS environment is sourced and set permissions at runtime
CMD ["bash", "-c", "\
sudo chown -R ${USERNAME}:${USERNAME} /opt/ros/noetic/flightSoftware && \
sudo chmod 666 /dev/i2c-1 && \
sudo chmod 666 /dev/gpiochip0 && \
source /opt/ros/noetic/setup.bash && \
if [ -f /opt/ros/noetic/flightSoftware/devel/setup.bash ]; then source /opt/ros/noetic/flightSoftware/devel/setup.bash; fi && \
xpra start --no-daemon --bind-ws=0.0.0.0:8080 --webcam=no --pulseaudio=no --mdns=no"]

