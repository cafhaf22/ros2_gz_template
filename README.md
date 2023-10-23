# ros_gz_project_template
A template project integrating ROS 2 and Gazebo simulator.

##*Note: This branch template was modified to work with ignition gazebo Fortress*
prior the use of the `gz` namespaces, this branch uses `ign` namespaces


## Included packages

* `ros_gz_example_description` - holds the sdf description of the simulated system and any other assets.

* `ros_gz_example_gazebo` - holds gazebo specific code and configurations. Namely this is where systems end up.

* `ros_gz_example_application` - holds ros2 specific code and configurations.

* `ros_gz_example_bringup` - holds launch files and high level utilities.


## Install
### Requirements

1. Choose a ROS and Gazebo combination  https://gazebosim.org/docs/harmonic/ros_installation
   Note: If you're using a specific and unsupported Gazebo version with ROS 2, you might need to set the `GZ_VERSION` environment variable, for example:

    ```bash
    export GZ_VERSION=garden
    ```

1. Install necessary tools

    ```bash
    sudo apt install python3-vcstool python3-colcon-common-extensions git wget
    ```

### Use as template
Directly `Use this template` and create your project repository on Github.

Or start by creating a workspace and cloning the template repository:

   ```bash
   mkdir -p ~/template_ws/src
   cd ~/template_ws/src
   wget https://raw.githubusercontent.com/gazebosim/ros_gz_project_template/main/template_workspace.yaml
   vcs import < template_workspace.yaml
   ```

## Usage

1. Install dependencies

    ```bash
    cd ~/template_ws
    source /opt/ros/<ROS_DISTRO>/setup.bash
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -i -y --rosdistro <ROS_DISTRO>
    ```

1. Build the project

    ```bash
    colcon build --cmake-args -DBUILD_TESTING=ON
    ```

1. Source the workspace

    ```bash
    . ~/template_ws/install/setup.sh
    ```

1. Launch the simulation

    ```bash
    ros2 launch ros_gz_example_bringup diff_drive.launch.py
    ```


## Useful commands

Move the smore robot

```bash
ros2 topic pub /smore/cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}"
```

Echo the odometry of the robot
```bash
ros2 topic echo /smore/odometry
```

For a more detailed guide on using this template see [documentation](https://gazebosim.org/docs/latest/ros_gz_project_template_guide).
