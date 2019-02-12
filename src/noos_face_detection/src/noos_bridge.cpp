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

sensor_msgs::Image mat2ros_image::operator()(cv::Mat img, 
                                                int counter)
{
    cv_bridge::CvImage out_msg;
    sensor_msgs::Image img_msg;

    std_msgs::Header header; // empty header
    header.seq = counter; // user defined counter
    header.stamp = ros::Time::now(); // time

    out_msg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
    out_msg.toImageMsg(img_msg);
    return img_msg;
}
