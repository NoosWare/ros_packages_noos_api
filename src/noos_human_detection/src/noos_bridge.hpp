#ifndef NOOS_BRIDGE_HPP
#define NOOS_BRIDGE_HPP

#include "includes.ihh"

/**
 * @struct mat_to_picture
 * @brief Doing the conversion from cv::Mat to noos::object::picture
 * is not trivial. With this struct the conversion is done
 * for .png images. If another extension is needed 
 * @version 0.1.0
 * @date 29.01.2019
 */
struct mat_to_picture
{
    noos::object::picture operator()(cv::Mat img);
};

/**
 * @struct human_convert2ros
 * @brief convert a noos::object::human to a human_detection::human msg
 * @version 0.1.0
 * @date 12.02.2019
 */
struct human_convert2ros
{
    human_detection::human operator()(noos::object::human noos_f);
};

/**
 * @struct mat2ros_image
 * @brief convert cv::Mat to sensor_msgs::Image
 * @version 31.01.2019
 * @version 0.1.0
 */
struct mat2ros_image
{
    sensor_msgs::Image operator()(cv::Mat img,
                                  int counter);
};

#endif
