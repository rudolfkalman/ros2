xhost +local:
sudo docker rm -f ros2_jazzy_gui
sudo docker run -p 6080:80 --security-opt seccomp=unconfined --shm-size=512m -v .:/home/ubuntu/projects/slam --net=host --ipc=host --env DISPLAY=$DISPLAY --volume /tmp/.X11-unix:/tmp/.X11-unix --name ros2_jazzy_gui ros2_jazzy
