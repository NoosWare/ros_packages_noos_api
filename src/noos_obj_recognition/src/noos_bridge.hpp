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
 * @struct data2pair_vector
 * @brief convert std::vector<std::pair<std::string, float>>
 *        into pair_vector data (custom msg)
 * @date 05.02.2019
 * @version 0.1.0
 */
struct data2pair_vector
{
    noos_obj_recognition::pair_vector operator()(std::vector<std::pair<std::string, float>> data);
};

#endif
