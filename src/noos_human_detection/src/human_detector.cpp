#include "human_detector.hpp"

human_detector::human_detector(noos::cloud::platform plat,
                               ros::NodeHandle node)
: callable_(std::bind(&human_detector::callback, this, std::placeholders::_1),
            plat),
  pub_(node.advertise<human_detection::humans>("humans", 1000)),
  pub_img_(node.advertise<human_detection::humans_cropped>("humans_cropped", 2000))
{}

void human_detector::detect(const cv::Mat & pict)
{
    image_ = pict;
    auto pic = mat_to_picture()(pict);
    if (!pic.type().empty()) {
        callable_.object = pic;
        callable_.send();
    }
    else {
        std::cout << "No image received" << std::endl;
    }
}

void human_detector::callback(std::vector<noos::object::human> humans)
{
    human_detection::humans total_humans;
    human_detection::humans_cropped imgs;
    for (auto each_human : humans) {
        total_humans.data.push_back(human_convert2ros()(each_human));
        auto cropped = image_(cv::Rect(each_human.top_left_x,
                                       each_human.top_left_y,
                                       each_human.bottom_right_x - each_human.top_left_x,
                                       each_human.bottom_right_y - each_human.top_left_y));
        imgs.data.push_back(mat2ros_image()(cropped,
                                            counter_));
    }
    pub_.publish(total_humans);
    pub_img_.publish(imgs);
    counter_++;
}

read_image::read_image(ros::NodeHandle node,
                       std::string topic_name)
: topic_(topic_name),
  cap_(0)
{
    if (topic_name.empty() ||
        topic_name == "none") {
        if(!cap_.isOpened()) {  // check if we succeeded
            throw std::runtime_error("No camera detected");
        }
    }
}

void read_image::take_image(const sensor_msgs::ImageConstPtr & msg)
{
    image_ = cv_bridge::toCvCopy(msg,
                                 sensor_msgs::image_encodings::BGR8)->image; 
}

void read_image::take_opencv_image()
{
    cap_ >> image_;
}

cv::Mat read_image::get_image()
{
    if (topic_.empty() ||
        topic_ == "none") {
        take_opencv_image();
    }
    return image_;
}
