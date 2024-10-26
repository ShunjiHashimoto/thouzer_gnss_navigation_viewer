FROM osrf/ros:humble-desktop-full

# キーボード設定の選択を省略
ARG DEBIAN_FRONTEND=noninteractive

# Install apt packages
RUN apt-get update && apt-get install -y \
git \
vim \
wget \
tmux \
terminator && \
apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update && apt-get install -y \
  python3-pip \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-rosbridge-library \
  ros-humble-urdf-launch \
  ros-humble-tf-transformations \
  mosquitto mosquitto-clients && \
  apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install paho-mqtt

ENV RMW_IMPLEMENTATION rmw_cyclonedds_cpp

# entrypoint.sh スクリプトをコンテナにコピー
COPY entrypoint.sh /entrypoint.sh

# スクリプトを実行可能にする
RUN chmod +x /entrypoint.sh

# スクリプトをエントリーポイントとして設定
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

# Create ROS workspace
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws/src
