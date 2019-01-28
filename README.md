# ros_wrapper_noos_api

Wrapper for using NOOS API with ROS

## Installation

```bash
git clone https://github.com/NoosWare/ros_packages_noos_api.git
cd ros_packages_noos_api
catkin_init_workspace
```

For building the packages, the [NOOS API](https://github.com/NoosWare/noos-api-cpp) is required.
If you have already installed, skip this step. If not, do the following:

```bash
git submodule update --init
cd noos_api/noos-api-cpp && mkdir build
cd build
cmake ..
make
sudo make install
```

Then, build the ROS packages:

```bash 
cd your_path/ros_packages_noos_api
catkin_make
```

## Running packages

Before running any package, it is needed to fill the file `config/platform.ini`
with your correct user and password of the Noos Cloud platform. If you don't have one,
register [here](https://noos.cloud/).

### ICP Slam

You will create a map of the enviroment, receiving the topic `/scan` (`sensor_msgs::LaserScan::ConstPtr`) 
the cloud will publish the pose of the robot in the topic `pose` (`geometry_msgs::Pose`).

The Noos Cloud needs a configuration file with the parameters about icp, for creating the map.
An example can be found in `/config/icp.ini`. It is the default file that will be loaded.

```bash
rosrun icp_slam icp_slam_node
```

That command will load the default parameters:
- icp config file = /config/icp.ini
- platform file = /config/platform.ini
- loaded = false

If you need to change one or more do the following:

```bash
rosrun icp_slam icp_slam_node args --icp your_path/icp_file.ini --platform your_path/configuration_file.ini --loaded true
```
