FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DOMAIN_ID=0

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    ffmpeg \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install ROS2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-suite \
    ros-humble-nav2-msgs \
    ros-humble-nav2-simple-commander \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-diagnostic-updater \
    ros-humble-interactive-markers \
    ros-humble-rviz2 \
    ros-humble-rosidl-generator-dds-idl \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# Install cyclonedds C library from source for unitree_sdk2_python
RUN apt-get update && apt-get install -y libacl1-dev uuid-dev \
    && cd /tmp && git clone --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git \
    && cd cyclonedds && mkdir build && cd build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF .. \
    && make -j$(nproc) && make install && rm -rf /tmp/cyclonedds \
    && rm -rf /var/lib/apt/lists/*

# Source ROS2 setup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
ENV CYCLONEDDS_HOME=/usr/local

# Build unitree_ros2 messages (provides unitree_go.msg)
RUN mkdir -p /root/unitree_ros2_ws/src && \
    cd /root/unitree_ros2_ws/src && \
    git clone --depth 1 https://github.com/unitreerobotics/unitree_ros2.git && \
    cp -r unitree_ros2/cyclonedds_ws/src/unitree/* . && \
    rm -rf unitree_ros2 && \
    cd /root/unitree_ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --packages-select unitree_go unitree_api unitree_hg && \
    echo "source /root/unitree_ros2_ws/install/setup.bash" >> ~/.bashrc

WORKDIR /app

# Copy requirements and install Python dependencies
COPY requirements-pip.txt .
RUN pip3 install --no-cache-dir -r requirements-pip.txt

# Install cyclonedds Python bindings and unitree_sdk2_python from git
RUN pip3 install --no-cache-dir cyclonedds==0.10.2

# Copy application code
COPY . .

# Install unitree_sdk2py from local source and fix missing __init__.py files
RUN find /app/src/unitree-sdk2py/unitree_sdk2py -type d -exec sh -c 'test -f "$1/__init__.py" || touch "$1/__init__.py"' _ {} \; && \
    pip3 install --no-cache-dir /app/src/unitree-sdk2py/

# Expose Gradio port
EXPOSE 7860

# Default command
CMD ["bash", "-c", "source /opt/ros/humble/setup.bash && source /root/unitree_ros2_ws/install/setup.bash && python3 main.py"]
