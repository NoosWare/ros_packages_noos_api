#include "qr_detector.hpp"

qr_detector::qr_detector(noos::cloud::platform plat,
                               ros::NodeHandle node)
: callable_(std::bind(&qr_detector::callback, this, std::placeholders::_1),
            plat),
  pub_(node.advertise<qr_detection::qrs>("qrs", 1000))
{}

void qr_detector::detect(const cv::Mat & pict)
{
    image_ = pict;
    auto pic = mat_to_picture()(pict);
    if (!pic.type().empty()) {
        callable_.object = pic;
        callable_.send(5);
    }
    else {
        std::cout << "No image received" << std::endl;
    }
}

void qr_detector::callback(std::vector<noos::object::qr_code> qrs)
{
    if (qrs.size() > 0) {
        qr_detection::qrs total_qrs;
        for (auto each_qr : qrs) {
            total_qrs.data.push_back(qr_convert2ros()(each_qr));
        }
        pub_.publish(total_qrs);
        counter_++;
    }
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
