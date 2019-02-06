#ifndef ORB_WRAPPER_HPP
#define ORB_WRAPPER_HPP

#include "noos_bridge.hpp"
#include "includes.ihh"
#include "options.hpp"

/**
 * @class orb_wrapper
 * @brief recognize the object you want in the image
 * @version 0.1.0
 * @date 06.02.2019
 */
class orb_wrapper
{
public:
    ///@brief constructor
    orb_wrapper(noos::cloud::platform plat,
                ros::NodeHandle node,
                orb_data data);

    ///@brief call the service detect faces
    void detect(const cv::Mat & pict);

private:
    // Upload image to the cloud
    void upload_image(std::string filename);

    //Callback
    void callback(std::vector<noos::object::point2d<float>> points);

    //Callback for uploading the image
    void upload_cb(bool result);

    //Publisher of the rectangles of faces
    ros::Publisher pub_;
	//platform
	noos::cloud::platform plat_;
    //orb data
    orb_data orb_data_;
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
