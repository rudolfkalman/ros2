FROM osrf/ros:jazzy-desktop-full

ARG USERNAME=ubuntu

# root
RUN apt-get update && apt-get install -y \
    sudo \
    python3-full \
    python3-venv \
    python3-pip \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-cartographer-rviz \
    ros-jazzy-imu-tools \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    neovim \
    tree \
 && rm -rf /var/lib/apt/lists/*

RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
 && chmod 0440 /etc/sudoers.d/ubuntu

# user
USER ${USERNAME}
WORKDIR /home/${USERNAME}/project

RUN python3 -m venv /home/${USERNAME}/pio-venv \
 && /home/${USERNAME}/pio-venv/bin/pip install -U pip \
 && /home/${USERNAME}/pio-venv/bin/pip install platformio

# bashrc config
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc \
 && echo "alias python='python3'" >> ~/.bashrc \
 && echo "alias vim='nvim'" >> ~/.bashrc \
 && echo "alias platformio='$HOME/pio-venv/bin/pio'" >> ~/.bashrc

# create platformio's venv penv
RUN rm -rf ~/.platformio/penv \
 && python3 -m venv ~/.platformio/penv

# install packages for mros build
RUN ~/.platformio/penv/bin/pip install \
  catkin-pkg \
  empy==3.3.4 \
  lark-parser \
  pyyaml \
  importlib-resources \
  markupsafe==2.0.1
