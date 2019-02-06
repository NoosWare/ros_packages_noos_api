#include "noos_bridge.hpp"

noos::object::picture mat_to_picture::operator()(cv::Mat img)
{
    std::vector<unsigned char> buf;
    cv::imencode(".png", img, buf);
    std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
    return noos::object::picture(conversion);
}

noos_orb::point_array data2point_array::operator()(std::vector<noos::object::point2d<float>> data)
{
    noos_orb::point_array msg;
    for (auto each_point : data) {
        geometry_msgs::Point32 point;
        point.x = each_point.x;
        point.y = each_point.y;
        point.z = 0;
        msg.points.push_back(point);
    }  
    return msg;
}
