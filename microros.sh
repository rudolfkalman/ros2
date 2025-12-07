# Serial micro-ROS Agent
sudo docker run -it --rm -v /dev:/dev --privileged --net=host --ipc=host microros/micro-ros-agent:kilted serial --dev /dev/ttyUSB0 -v6
