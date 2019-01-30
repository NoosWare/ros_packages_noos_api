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

They are independent between them, so you can add the package that you need to your workspace
in case you don't want to build all of them, or you don't have installed all the dependencies.

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

### Chatbot

You can have a conversation with the chatbot sending sentences through the topic `talker` and the answer will be 
published in `chatter` topic.

```bash 
rosrun chatbot chatbot_node
```

### Face Detection

It will detect faces on images. The images can be sent to the node with a `topic` of your election (it just need to send 
`sensor_msgs::Image` type) or taking images from the webcam directly.

In the first case, the name of the topic needs to be known by the node:

```bash
rosrun face_detection face_detection --topic your_topic_name
```

If you prefer to use a webcam:

```bash 
rosrun face_detection face_detection_node
```

It will publish the location of the faces in the image with custom ros message `faces` in the topic `/faces`:

- face.msg

```bash
float32 top_left_x
float32 top_left_y
float32 bottom_right_x
float32 bottom_right_y
```

- faces.msg

```bash 
face[] data
```


