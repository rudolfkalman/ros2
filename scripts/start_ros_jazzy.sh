#!/bin/bash

sudo docker compose up -d ros2

sudo docker ps

sudo docker exec -it ros2-jazzy /bin/bash
