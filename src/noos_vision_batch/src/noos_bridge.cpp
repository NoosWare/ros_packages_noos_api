#include "noos_bridge.hpp"

void batch_data::clean()
{
    age.clear();
    gender.clear();
    expression.clear();
}

bool batch_data::check_all_filled()
{
    if (age.size() != 0 &&
        gender.size() != 0 &&
        expression.size() != 0) {
        return true;
    }
    return false;
}

noos::object::picture mat_to_picture::operator()(cv::Mat img)
{
    std::vector<unsigned char> buf;
    cv::imencode(".png", img, buf);
    std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
    return noos::object::picture(conversion);
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

noos_vision_batch::pair_vector data2pair_vector::operator()(std::vector<std::pair<std::string, float>> data)
{
    noos_vision_batch::pair_vector msg;
    for (auto each_pair : data) {
        noos_vision_batch::pair custom_pair;
        custom_pair.probability = each_pair.second;
        custom_pair.data = each_pair.first;
        msg.pairs.push_back(custom_pair);
    }  
    return msg;
}

noos_vision_batch::batch pair_vector2batch::operator()(int face_number,
                                                  noos_vision_batch::pair_vector age,
                                                  noos_vision_batch::pair_vector gender,
                                                  noos_vision_batch::pair_vector expression)
{
    noos_vision_batch::batch msg;
    msg.age = age;
    msg.gender = gender;
    msg.expressions = expression;
    msg.face_number = face_number;
    return msg;
}

