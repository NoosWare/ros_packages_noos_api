#include "vision_batch.hpp"

vision_batch::vision_batch(noos::cloud::platform plat,
                               ros::NodeHandle node)
: exp_tie_([&](const auto data){ this->face_expression_cb(data);}),
  age_tie_([&](const auto data) { this->age_detection_cb(data);}),
  gender_tie_([&](const auto data) { this->age_detection_cb(data);}),
  pub_(node.advertise<face_detection::faces>("batch", 1000)),
{}

void vision_batch::get_image(const vision_batch::faces_cropped & faces);
{
    //image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image; 
}

void vision_batch::face_expression_cb(std::vector<std::pair<std::string,float>> data)
{
    if (!data.empty()) {
        all_data_.expression = data;
    }
}

void vision_batch::age_detection_cb(std::vector<std::pair<std::string,float>> data)
{
    if (!data.empty()) {
        all_data_.age = data;
    }
}

void vision_batch::gender_detection_cb(std::vector<std::pair<std::string,float>> data)
{
    if (!data.empty()) {
        all_data_.gender = data;
    }
}
