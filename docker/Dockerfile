FROM ros:jazzy-ros-base

ARG USER_ID=1000
ARG GROUP_ID=1000

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_BREAK_SYSTEM_PACKAGES=1
ENV PIP_NO_CACHE_DIR=1
ENV MUJOCO_VERSION=3.3.6
ENV MUJOCO_HOME=/opt/mujoco/mujoco-3.3.6
ENV UNITREE_MUJOCO_DIR=/home/user/workspace/third_party/unitree_mujoco
ENV UNITREE_RL_LAB_DIR=/home/user/workspace/third_party/unitree_rl_lab
ENV G1_SIM_AUTOBUILD=1
ENV G1_SIM_AUTOBUILD_RL_CONTROLLER=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV CYCLONEDDS_URI=file:///etc/cyclonedds_config.xml

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && apt-get install -y --no-install-recommends \
    sudo \
    build-essential \
    ca-certificates \
    cmake \
    curl \
    git \
    gosu \
    libboost-all-dev \
    libeigen3-dev \
    libgl1 \
    libgl1-mesa-dri \
    libglfw3-dev \
    libpcl-dev \
    libspdlog-dev \
    libx11-6 \
    libxcursor1 \
    libxinerama1 \
    libxi6 \
    libxrandr2 \
    libxxf86vm1 \
    libyaml-cpp-dev \
    nano \
    python3-colcon-common-extensions \
    python3-pip \
    python3-setuptools \
    python3-venv \
    python3-yaml \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-rosidl-default-generators \
    ros-jazzy-rosidl-generator-dds-idl \
    ros-jazzy-ros2launch \
    ros-jazzy-rviz2 \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-tf2-eigen \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-xacro \
    xdotool && \
    rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --ignore-installed PyYAML \
    openai \
    pydantic \
    pydantic-ai

RUN git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2.git /tmp/unitree_sdk2 && \
    cmake -S /tmp/unitree_sdk2 -B /tmp/unitree_sdk2/build -DCMAKE_INSTALL_PREFIX=/usr/local && \
    cmake --build /tmp/unitree_sdk2/build -j"$(nproc)" && \
    cmake --install /tmp/unitree_sdk2/build && \
    rm -rf /tmp/unitree_sdk2

RUN mkdir -p /opt/mujoco && \
    curl -L "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" -o /tmp/mujoco.tar.gz && \
    tar -xzf /tmp/mujoco.tar.gz -C /opt/mujoco && \
    rm /tmp/mujoco.tar.gz

RUN touch /var/lib/lastlog && \
    if getent passwd ${USER_ID}; then userdel -f $(getent passwd ${USER_ID} | cut -d: -f1); fi && \
    if getent group ${GROUP_ID}; then groupdel $(getent group ${GROUP_ID} | cut -d: -f1); fi || true

RUN groupadd -g ${GROUP_ID} user && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

WORKDIR /home/user/workspace

RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash.bashrc && \
    echo "if [ -f /home/user/workspace/install/setup.bash ]; then source /home/user/workspace/install/setup.bash; fi" >> /etc/bash.bashrc && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash'" >> /etc/bash.bashrc && \
    echo "alias cs='source install/setup.bash'" >> /etc/bash.bashrc

COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/cyclonedds_config.xml /etc/cyclonedds_config.xml
RUN chmod +x /entrypoint.sh

USER user
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
