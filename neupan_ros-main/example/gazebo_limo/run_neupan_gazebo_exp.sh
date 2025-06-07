#!/bin/bash

# Step 1: Launch the Gazebo environment
gnome-terminal -- bash -c "roslaunch neupan_ros gazebo_limo_env_complex_20.launch"

# Give some time for Gazebo to start
sleep 15

# Step 2: Launch the NeuPAN control with obstacle information from laser scan
gnome-terminal -- bash -c "roslaunch neupan_ros neupan_gazebo_limo.launch"