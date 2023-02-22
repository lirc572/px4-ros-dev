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

# Note that if you want to run the Gazebo GUI, you need to run a X server on the host machine, and set the environment variable `DISPLAY` to the IP address of your host machine within the container. We recommend using MobaXterm on Windows, which has a built-in X server. You can find the IP address by hovering over the X server icon on the top right corner of the MobaXterm GUI. Then run the following command in the container before running `make px4_sitl gazebo`:
export DISPLAY=YOUR_IP:0.0

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

### libGL error: failed to load driver: swrast

If you see `libGL error: failed to load driver: swrast` while trying to start Gazebo, you are likely using an Nvidia GPU. You need to install the Nvidia driver in the container first. It is recommended to use an Intel integrated GPU which works by default.

```bash
`apt-get update && apt-get install -y nvidia-driver-440`
```


**The steps below is suggested by the PX4 official docs, but it didn't work for me**
1. Download the driver from <https://www.nvidia.com/download/index.aspx>
2. `apt-get update && apt-get install -y kmod`
3. `./NVIDIA-DRIVER.run -a -N --ui=none --no-kernel-module` (replace `NVIDIA-DRIVER.run` with the name of the file you downloaded, you may need to set the executable bit first with `chmod +x NVIDIA-DRIVER.run`)

## References

- [ROS1 Tutorials](https://wiki.ros.org/ROS/Tutorials)
- [PX4 Docs on MAVROS](https://docs.px4.io/main/en/ros/mavros_offboard_python.html)
- [PX4 Docs on Docker Containers](https://docs.px4.io/main/en/test_and_ci/docker.html)
