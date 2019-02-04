#include "vision_batch_wrapper.hpp"

vision_batch_wrapper::vision_batch_wrapper(noos::cloud::platform plat,
                                           ros::NodeHandle node)
: exp_tie_([&](const auto data){ this->face_expression_cb(data);}),
  age_tie_([&](const auto data) { this->age_detection_cb(data);}),
  gender_tie_([&](const auto data) { this->gender_detection_cb(data);}),
  plat_(plat),
  pub_(node.advertise<noos_vision_batch::batch>("batch", 1000))
{}

void vision_batch_wrapper::get_image(noos_vision_batch::faces_cropped msg)
{
    for (auto img : msg.data) {
        finished = false;
        auto image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8)->image; 
        send_info(mat_to_picture()(image));
        if (!finished) {
            ros::Duration(0.1).sleep();
        }
    }
}

void vision_batch_wrapper::face_expression_cb(std::vector<std::pair<std::string,float>> data)
{
    if (!data.empty()) {
        all_data_.expression = data;
        if (all_data_.check_all_filled()) {
            publish_result();
        }
    }
}

void vision_batch_wrapper::age_detection_cb(std::vector<std::pair<std::string,float>> data)
{

    if (!data.empty()) {
        all_data_.age = data;
        if (all_data_.check_all_filled()) {
            publish_result();
        }
    }
}

void vision_batch_wrapper::gender_detection_cb(std::vector<std::pair<std::string,float>> data)
{
    if (!data.empty()) {
        all_data_.gender = data;
        if (all_data_.check_all_filled()) {
            publish_result();
        }
    }
}

void vision_batch_wrapper::send_info(noos::object::picture image)
{

    if (!batch_) {
        batch_ = std::make_unique<noos::cloud::callable<vbatch,true>>(image, 
                                                                       plat_, 
                                                                       exp_tie_,
                                                                       age_tie_,
                                                                       gender_tie_); 
    } 
    else {
        batch_->object = vbatch(image, exp_tie_, age_tie_, gender_tie_);
    }
    assert(batch_);
    batch_->send();
}

void vision_batch_wrapper::publish_result()
{
    counter++;
    pub_.publish(pair_vector2batch()(counter,
                                    data2pair_vector()(all_data_.age), 
                                    data2pair_vector()(all_data_.gender), 
                                    data2pair_vector()(all_data_.expression)));
    all_data_.clean();
    finished = true;
}
