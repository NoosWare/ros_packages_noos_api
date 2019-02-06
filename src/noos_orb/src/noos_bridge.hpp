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
 * @struct data2point_array
 * @brief convert std::vector<noos::object::point2d<float>>
 *        into point_array data (custom msg)
 * @date 06.02.2019
 * @version 0.1.0
 */
struct data2point_array
{
    noos_orb::point_array operator()(std::vector<noos::object::point2d<float>> data);
};

#endif
