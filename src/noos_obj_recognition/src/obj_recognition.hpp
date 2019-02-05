#ifndef OBJ_RECOGNITION_HPP
#define OBJ_RECOGNITION_HPP

#include "noos_bridge.hpp"
#include "includes.ihh"

/**
 * @class obj_recognition
 * @brief recognize the object in the image
 * @version 0.1.0
 * @date 05.02.2019
 */
class obj_recognition
{
public:
    ///@brief constructor
    obj_recognition(noos::cloud::platform plat,
                    ros::NodeHandle node);

    ///@brief call the service detect faces
    void detect(const cv::Mat & pict);

private:
    //Callback
    void callback(std::vector<std::pair<std::string, float>> obj);
    //Publisher of the rectangles of faces
    ros::Publisher pub_;
	//platform
	noos::cloud::platform plat_;
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
