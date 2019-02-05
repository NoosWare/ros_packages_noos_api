#include "obj_recognition.hpp"

obj_recognition::obj_recognition(noos::cloud::platform plat,
                               ros::NodeHandle node)
: pub_(node.advertise<noos_obj_recognition::pair_vector>("object_recognition", 1000)),
  plat_(plat)
{}

void obj_recognition::detect(const cv::Mat & pict)
{
    if (pict.empty()) {
        return;
    }
    image_ = pict;
    auto pic = mat_to_picture()(pict);
    if (!pic.type().empty()) {
        noos::cloud::callable<noos::cloud::object_recognition, 
                              false> call(std::bind(&obj_recognition::callback, this, std::placeholders::_1),
                                          plat_,
                                          pic);
        call.send();
    }
    else {
        std::cout << "error with image" << std::endl;
    }
}

void obj_recognition::callback(std::vector<std::pair<std::string, float>> obj)
{
	pub_.publish(data2pair_vector()(obj));
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
    auto copy = image_;
    image_.release();
    return copy;
}
