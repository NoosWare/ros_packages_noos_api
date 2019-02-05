# Ros_packages_noos_api

Ros packages to interact with [Noos Cloud](https://noos.cloud)

<img src="https://github.com/NoosWare/List_repositories/blob/master/images/Noos.png" width="200" height="120" />

## Installation

```bash
git clone https://github.com/NoosWare/ros_packages_noos_api.git
cd ros_packages_noos_api
catkin_init_workspace
```

For building the packages, the [NOOS API](https://github.com/NoosWare/noos-api-cpp) is required.
If you have already installed it, skip this step. If not, do the following:

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

**NOTE:** Using your user and password, you only need to upload the `icp config file` once. It will be
saved in the cloud, so you don't have to upload it every time you want to run the program.

```bash
rosrun icp_slam icp_slam_node
```

That command will load the default parameters:
- `icp config file` = /config/icp.ini
- `platform file` = /config/platform.ini
- `loaded` = false  (if the icp config file has been uploaded previously)

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

It will publish the location of the faces in the image with custom ros message `faces` in the topic `/faces`
and the face cropped from the image (in the case you don't have access to the original image) in the topic `/faces_cropped`:

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

- faces_cropped.msg

```bash 
Header header
sensor_msgs/Image[] data 
```

### Vision Batch

Given a cropped face, it will send back the age, gender and facial expression of that face.
The node is subscribed to a topic given (using the option --topic your_name_topic), and 
it required a `face_cropped` image, which is:

```bash 
Header header
sensor_msgs/Image[] data 
```

The result is given in a `batch.msg`:

```bash 
int32 face_number
pair_vector age
pair_vector gender
pair_vector expressions
```

- pair_vector.msg

```bash
pair[] pairs
```

- pair.msg

```bash
float32 probability
string data
```

Example of the data:

```bash
face_number: 20
age:
  pairs:
    -
      probability: 0.872070670128
      data: "20-30"
    -
      probability: 0.1253926754
      data: "30-46"
gender:
  pairs:
    -
      probability: 0.997378945351
      data: "female"
expressions:
  pairs:
    -
      probability: 0.780185818672
      data: "fear"
    -
      probability: 0.155177041888
      data: "neutral"
---
```

**To run the node:**

```bash
rosrun noos_vision_batch noos_vision_batch_node --topic name_of_your_topic
```

If you want to test it, you can use `face_detection_node` as image creator. So after running that node,
use the following line to run vision_batch:

```bash
rosrun noos_vision_batch noos_vision_batch_node --topic faces_cropped
```

### Object Recognition

It will detect the object in the image. The images can be sent to the node with a `topic` of your election (it just need to send 
`sensor_msgs::Image` type) or taking images from the webcam directly.

In the first case, the name of the topic needs to be known by the node:

```bash
rosrun noos_obj_recognition noos_obj_recognition --topic your_topic_name
```

If you prefer to use a webcam:

```bash 
rosrun noos_obj_recognition noos_obj_recognition_node
```

It will publish a `pair_vector` msg with the object and the probability.

- pair_vector.msg

```bash
pair[] pairs
```

- pair.msg

```bash
float32 probability
string data
```

Example of the result obtained:

```bash
pairs:
  -
    probability: 0.0327134691179
    data: "bucket, pail"
  -
    probability: 0.0266536492854
    data: "coffee mug"
  -
    probability: 0.0419563986361
    data: "coffeepot"
  -
    probability: 0.0197438318282
    data: "hair spray"
  -
    probability: 0.036662440747
    data: "lighter, light, igniter, ignitor"
  -
    probability: 0.0375811494887
    data: "lipstick, lip rouge"
  -
    probability: 0.0140977697447
    data: "maraca"
  -
    probability: 0.0309944115579
    data: "paintbrush"
```


