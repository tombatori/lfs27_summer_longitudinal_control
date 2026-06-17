# Ubuntu 22.04 like real carrrr
FROM ubuntu:22.04

# Set the environment to non-interactive for automated installation
ENV DEBIAN_FRONTEND=noninteractive

# Update package list and install basic dependencies
RUN apt update && apt install -y \
    software-properties-common \
    curl \
    wget \
    build-essential \
    git \
    cmake \
    libeigen3-dev \
    libjsoncpp-dev \
    libspdlog-dev \
    libcurl4-openssl-dev \
    libssl-dev \
    python3-pip

# Add ROS2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list' && \
    apt update && apt install -y ros-humble-desktop

# Install additional ROS2 and other dependencies
RUN apt install -y \
    ros-humble-tf2-eigen \
    ros-humble-foxglove-bridge \
    python3-colcon-common-extensions

# Source ROS2 setup script for all installed versions
RUN for i in `ls /opt/ros/`; do echo "source /opt/ros/$i/setup.bash" >> ~/.bashrc; done

RUN echo "source /workspace/.bashrc" >> ~/.bashrc

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> ~/.bashrc

# # Set environment variable for ROS2 implementation
# ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc

# Set entrypoint to source the bashrc when the container starts
ENTRYPOINT ["/bin/bash", "-c", "source ~/.bashrc && exec \"$@\"", "--"]


