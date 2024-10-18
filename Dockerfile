FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV USERNAME=user
ARG user_id=2000  # Default value, can be overridden by docker-compose.yml

# Switch to a non-root user
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
    && echo "$USERNAME:$USERNAME" | chpasswd \
    && adduser $USERNAME sudo \
    && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Base dependencies
RUN sudo apt-get update
RUN sudo apt-get install -y --no-install-recommends \
    libserial-dev \
    libeigen3-dev \
    ros-noetic-tf \
    ros-noetic-serial \
    netbase \
    ros-noetic-vrpn \
    ros-noetic-vrpn-client-ros \
    iputils-ping \
    setserial \
    libreadline-dev

# Copy your ROS1 workspace into the container
COPY ./flightSoftware /opt/ros/noetic/flightSoftware/src

# Change ownership after copying the files
RUN sudo chown -R 2000:2000 /opt/ros/noetic/flightSoftware

USER $USERNAME

# Serial access from regular user
RUN sudo usermod -aG dialout $USERNAME

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /opt/ros/noetic/flightSoftware/devel/setup.bash ]; then source /opt/ros/noetic/flightSoftware/devel/setup.bash; fi" >> ~/.bashrc

# Set the entrypoint and ensure ROS environment is sourced
CMD ["bash", "-c", "source /opt/ros/noetic/setup.bash && \
                    if [ -f /opt/ros/noetic/flightSoftware/devel/setup.bash ]; then source /opt/ros/noetic/flightSoftware/devel/setup.bash; fi && exec bash"]
