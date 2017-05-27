# Crash Racing Team ROS Workspace

For more information see the main repository `crt`.

## Dependencies

* ROS Kinetic

To install dependencies of the included packages:

```
$ rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
```

## Build

Use `catkin_make` with a single thread as there seem to be some dependency problems:

```
$ catkin_make -j1
```

## License

All of our own work, especially new code and changes, is licensed under GPLv3.

## Credits

Thanks especially to the following Open Source Projects!

* `crt_control` based on: https://github.com/davetcoleman/ros_control_boilerplate (BSD)
* configuration files of `crt_navigation` based on: https://github.com/cvra/goldorak
* `crt_description` based on: https://github.com/ros-simulation/gazebo_ros_demos (GPLv3)

We include the following packages mostly unmodified for convenience:

* `hokuyo_node` from: https://github.com/ros-drivers/hokuyo_node (LGPL) (we had some problems using the newer `urg_node`)
* `raspicam_node` from: https://github.com/UbiquityRobotics/raspicam_node (BSD) (pi camera not really used for anything, but looks good in rviz)
* `driver_common` from: https://github.com/ros-drivers/driver_common.git (BSD)
* `lcsr_tf_tools` from: https://github.com/jhu-lcsr/lcsr_tf_tools (BSD) (for its multistatic publisher)
