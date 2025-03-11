FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV USERNAME=user
ARG user_id=2000  # Default value, can be overridden by docker-compose.yml

# Base dependencies installation (including i2c-tools)
RUN apt-get update && apt-get install -y --no-install-recommends \
    libserial-dev \
    libeigen3-dev \
    ros-noetic-tf \
    ros-noetic-serial \
    git \
    python3-pip \
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

# Copy your ROS1 workspace into the container
COPY ./flightSoftware /opt/ros/noetic/flightSoftware/src

USER $USERNAME
RUN pip install git+https://github.com/hungpham2511/toppra

# Update .bashrc to source ROS environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /opt/ros/noetic/flightSoftware/devel/setup.bash ]; then source /opt/ros/noetic/flightSoftware/devel/setup.bash; fi" >> ~/.bashrc

# Entry point to ensure ROS environment is sourced and set permissions at runtime
CMD ["bash", "-c", "sudo chown -R ${USERNAME}:${USERNAME} /opt/ros/noetic/flightSoftware && \
                    sudo chmod 666 /dev/i2c-1 && \
                    sudo chmod 666 /dev/gpiochip0 && \
                    source /opt/ros/noetic/setup.bash && \
                    if [ -f /opt/ros/noetic/flightSoftware/devel/setup.bash ]; then source /opt/ros/noetic/flightSoftware/devel/setup.bash; fi && exec bash"]
