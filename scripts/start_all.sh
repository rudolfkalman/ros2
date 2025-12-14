#!/bin/bash

sudo docker compose down

sudo docker compose up -d

sudo docker ps

sudo docker exec -it ros2-jazzy /bin/bash
