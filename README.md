# px4-docker-dev

This repository provides a Docker based development environment for PX4 and ROS. It also provides a Nvidia Jetson based environment for testing on an actual companion computer.

## Development Environment

> **Creating a ROS Package**: follow the instructions in [this section](#create-a-ros-package).

```bash
cd dev
docker compose -f docker/docker-compose.yml up --build

# In a new terminal
docker exec -it px4-ros-dev-px4-1 bash
export DISPLAY=:0
make px4_sitl gazebo # or make px4_sitl_default gazebo
# Use a VNC client to connect to the VNC server at 127.0.0.1:5900, the password is `password`

# In a new terminal
docker exec -it px4-ros-dev-ros-1 bash
cd catkin_ws
source devel/setup.bash
roslaunch package_name launch_file_name.launch
```

## Jetson Environment

> **Creating a ROS Package**: follow the instructions in [this section](#create-a-ros-package).

Open `jetson/docker/docker-compose.yml` and update the `devices` section of the `ros` service to match the actual device path of the serial port on your Jetson that is connected to the Pixhawk. You can find the device path by running `ls /dev/tty*` on the Jetson. Then run the following command to start the containers:

```bash
cd jetson
docker-compose -f docker/docker-compose.yml up --build

# In a new terminal
docker exec -it px4-ros-jetson-ros-1 bash
cd catkin_ws
source devel/setup.bash
roslaunch package_name launch_file_name.launch
```

## Create a ROS Package

```bash
# If you want to create a fresh ROS environment, continue with the instructions below, otherwise skip to the comment with a line of hashes

# Remove the existing ROS workspace directory
rm -rf dev/ros # or rm -rf jetson/ros if you are using the Jetson environment

# Create the directory structure for the ROS workspace
mkdir -p dev/ros/catkin_ws/src # or mkdir -p jetson/ros/catkin_ws/src if you are using the Jetson environment

################################################################################
# Start the containers by following the instructions in the above sections

# In a new terminal
# Attach a shell to the ROS container
docker exec -it px4-ros-dev-ros-1 bash # or docker exec -it px4-ros-jetson-ros-1 bash if you are using the Jetson environment
source /opt/ros/melodic/setup.bash

# Create a new ROS workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Create a new ROS package
roscd  # Should cd into ~/catkin_ws/devel
cd ../src
catkin_create_pkg package_name rospy

# Build the workspace (to build our new package)
cd ..
catkin_make

# From here you can do whatever you would normally do with a ROS package
```

Since the ROS container uses the root user by default, the files created by the container will be owned by root and you will receive permission errors when trying to edit the files outside the container

To be able to edit the files outside the container, you need to change the owner of the files to your user:

```bash
sudo chown -R $USER:$USER dev/ros/catkin_ws # or sudo chown -R $USER:$USER jetson/ros/catkin_ws if you are using the Jetson environment
```

## Troubleshooting

### UAS: GeographicLib exception

If you encounter `UAS: GeographicLib exception` while running the ROS package, it may be because the `geographiclib` package is not correctly installed during the Docker build process. You can manually run `curl -fsSL https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | bash` in the ROS container to install the package.

## References

- [ROS1 Tutorials](https://wiki.ros.org/ROS/Tutorials)
- [PX4 Docs on MAVROS](https://docs.px4.io/main/en/ros/mavros_offboard_python.html)
- [PX4 Docs on Docker Containers](https://docs.px4.io/main/en/test_and_ci/docker.html)
