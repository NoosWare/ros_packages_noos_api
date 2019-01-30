#ifndef FACE_DETECTOR_HPP
#define FACE_DETECTOR_HPP

#include "noos_bridge.hpp"
#include "includes.ihh"

/**
 * @class face_detector
 * @brief detect faces in an image
 * @version 0.1.0
 * @date 29.01.2019
 */
class face_detector
{
public:
    ///@brief constructor
    face_detector(noos::cloud::platform plat,
                  ros::NodeHandle node);

    ///@brief call the service detect faces
    void detect(const noos::object::picture & pic);

private:
    //Callback
    void callback(std::vector<noos::object::face> faces);
    //Callable object
    noos::cloud::callable<noos::cloud::face_detection, true> callable_;
    //Publisher
    ros::Publisher pub_;
};

/**
 * @class read_image
 * @brief Depending of the user necessities, the image can be read from
 *        a webcam or from a topic
 * @version 0.1.0
 * @date 30.01.2019
 */
class read_image
{
public:
    ///@brief Constructor
    read_image(ros::NodeHandle node,
               std::string topic_name);

    ///@brief get the image cv::Mat format
    cv::Mat get_image();
    ///@brief  get the image from ros topic
    void take_image(const sensor_msgs::ImageConstPtr & msg);

private:
    // get the image from webcam
    void take_opencv_image();
    //name of the topic
    std::string topic_;
    //video capture
    cv::VideoCapture cap_;
    // Image
    cv::Mat image_;
};

#endif
