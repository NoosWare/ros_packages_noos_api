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
 * @struct qr_convert2ros
 * @brief convert a noos::object::qr to a qr_detection::qr msg
 * @version 0.1.0
 * @date 12.02.2019
 */
struct qr_convert2ros
{
    qr_detection::qr operator()(noos::object::qr_code noos_qr);
};

#endif
