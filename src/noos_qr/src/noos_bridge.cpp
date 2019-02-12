#include "noos_bridge.hpp"

noos::object::picture mat_to_picture::operator()(cv::Mat img)
{
    std::vector<unsigned char> buf;
    cv::imencode(".png", img, buf);
    std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
    return noos::object::picture(conversion);
}

qr_detection::qr qr_convert2ros::operator()(noos::object::qr_code noos_qr)
{
    qr_detection::qr ros_qr;

    ros_qr.center_x = noos_qr.centre_x;
    ros_qr.center_y = noos_qr.centre_y;
    ros_qr.message = noos_qr.message;

    return ros_qr;
}

