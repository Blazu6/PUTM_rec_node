#!/bin/bash

BAG_DIR="/home/putm/rosbagservice"

mkdir -p "$BAG_DIR"

exec ros2 bag record -a --max-bag-duration 1800 -o "$BAG_DIR/rosbag_$(date +%Y-%m-%d_%H-%M-%S)"
