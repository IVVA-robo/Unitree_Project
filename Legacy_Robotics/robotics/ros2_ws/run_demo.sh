#!/bin/bash

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
setup_file="$workspace_dir/install/setup.bash"
description_share="$workspace_dir/install/unitree_r1_description/share"

gnome-terminal -- bash -c "source '$setup_file'; export GAZEBO_MODEL_DATABASE_URI=''; export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:'$description_share'; ros2 launch unitree_r1_description gazebo.launch.py; exec bash"
gnome-terminal -- bash -c "source '$setup_file'; ros2 run joint_state_publisher_gui joint_state_publisher_gui; exec bash"
gnome-terminal -- bash -c "source '$setup_file'; rviz2; exec bash"
