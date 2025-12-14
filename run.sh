xhost +local:
sudo docker rm -f ros2_jazzy
sudo docker run -it --name ros2_jazzy --privileged --net=host --ipc=host -v /dev:/dev -v .:/home/ubuntu/project/ --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros-jazzy
