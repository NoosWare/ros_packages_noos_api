#include "noos_bridge.hpp"

noos::object::picture mat_to_picture::operator()(cv::Mat img)
{
    std::vector<unsigned char> buf;
    cv::imencode(".png", img, buf);
    std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
    return noos::object::picture(conversion);
}

face_detection::face face_convert2ros::operator()(noos::object::face noos_f)
{
    face_detection::face ros_f;

    ros_f.top_left_x = noos_f.top_left_x;
    ros_f.top_left_y = noos_f.top_left_y;
    ros_f.bottom_right_x = noos_f.bottom_right_x;
    ros_f.bottom_right_y = noos_f.bottom_right_y;

    return ros_f;
}
