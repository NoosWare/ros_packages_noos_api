#include "noos_bridge.hpp"

noos::object::picture mat_to_picture::operator()(cv::Mat img)
{
    std::vector<unsigned char> buf;
    cv::imencode(".png", img, buf);
    std::vector<noos::types::byte> conversion(buf.begin(), buf.end());
    return noos::object::picture(conversion);
}

noos_obj_recognition::pair_vector data2pair_vector::operator()(std::vector<std::pair<std::string, float>> data)
{
    noos_obj_recognition::pair_vector msg;
    for (auto each_pair : data) {
        noos_obj_recognition::pair custom_pair;
        custom_pair.probability = each_pair.second;
        custom_pair.data = each_pair.first;
        msg.pairs.push_back(custom_pair);
    }  
    return msg;
}
