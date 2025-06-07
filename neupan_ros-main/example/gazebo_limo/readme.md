# Example in Gazebo with LIMO

## Prequisites

This example relies on following packages:

- [rvo_ros](https://github.com/hanruihua/rvo_ros)
- [limo_ros](https://github.com/hanruihua/limo_ros)

You can install these repositories manually or by running the provided shell script:

```bash
sh gazebo_example_setup.sh
```

## Run Gazebo Example

Step 1. Launch Gazebo environment with multiple nonconvex moving obstacles

```bash
roslaunch neupan_ros gazebo_limo_env_complex_20.launch
```

Step 2. Launch Neupan Controller

```bash
roslaunch neupan_ros neupan_gazebo_limo.launch
```

or run the following script to launch the environment and the neupan controller at once

```bash
./run_neupan_gazebo_exp.sh
```

**Note**: you can change the goal by clicking rviz 2D nav goal.








