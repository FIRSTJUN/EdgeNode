FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    nano \
    vim \
    net-tools \
    iputils-ping \
    build-essential \
    ros-humble-demo-nodes-cpp \
    ros-humble-demo-nodes-py \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'if [ -f /workspace/ros2_ws/install/setup.bash ]; then' \
    '  source /workspace/ros2_ws/install/setup.bash' \
    'fi' \
    >> /root/.bashrc

CMD ["sleep", "infinity"]
ARG USER_UID=1000
ARG USER_GID=1000

RUN if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} dev; \
    fi && \
    if ! getent passwd ${USER_UID} >/dev/null; then \
        useradd --uid ${USER_UID} --gid ${USER_GID} \
        --create-home --shell /bin/bash dev; \
    fi
RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    '[ -f /workspace/ros2_ws/install/setup.bash ] && source /workspace/ros2_ws/install/setup.bash' \
    >> /home/dev/.bashrc
