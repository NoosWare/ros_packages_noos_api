#ifndef HUMAN_DETECTOR_HPP
#define HUMAN_DETECTOR_HPP

#include "noos_bridge.hpp"
#include "includes.ihh"

/**
 * @class human_detector
 * @brief detect humans in an image
 * @version 0.1.0
 * @date 12.02.2019
 */
class human_detector
{
public:
    ///@brief constructor
    human_detector(noos::cloud::platform plat,
                  ros::NodeHandle node);

    ///@brief call the service detect humans
    void detect(const cv::Mat & pict);

private:
    //Callback
    void callback(std::vector<noos::object::human> humans);
    //Callable object
    noos::cloud::callable<noos::cloud::human_detection, true> callable_;
    //Publisher of the rectangles of humans
    ros::Publisher pub_;
    //Publisher of the image cropped
    ros::Publisher pub_img_;
    //frame_id
    int counter_ = 0;
    //cv::Mat image
    cv::Mat image_;
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
