#!/bin/bash
docker build --build-arg user_id=$(id -u) --rm -t ros-jazzy:cuda -f Dockerfile.jazzy.cuda .