# Serial micro-ROS Agent
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host microros/micro-ros-agent:rolling udp4 --port 8888 -v6
