xhost +local:
sudo docker rm -f ros2_jazzy_gui
sudo docker run -it --name ros2_jazzy_gui --privileged --net=host --ipc=host -v /dev:/dev -v .:/home/ubuntu/projects/slam --env DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix ros2_jazzy
