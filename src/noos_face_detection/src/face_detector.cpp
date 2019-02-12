#include "face_detector.hpp"

face_detector::face_detector(noos::cloud::platform plat,
                               ros::NodeHandle node)
: callable_(std::bind(&face_detector::callback, this, std::placeholders::_1),
            plat),
  pub_(node.advertise<face_detection::faces>("faces", 1000)),
  pub_img_(node.advertise<face_detection::faces_cropped>("faces_cropped", 2000))
{}

void face_detector::detect(const cv::Mat & pict)
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

void face_detector::callback(std::vector<noos::object::face> faces)
{
    face_detection::faces total_faces;
    face_detection::faces_cropped imgs;
    for (auto each_face : faces) {
        total_faces.data.push_back(face_convert2ros()(each_face));
        auto cropped = image_(cv::Rect(each_face.top_left_x,
                                       each_face.top_left_y,
                                       each_face.bottom_right_x - each_face.top_left_x,
                                       each_face.bottom_right_y - each_face.top_left_y));
        imgs.data.push_back(mat2ros_image()(cropped,
                                            counter_));
    }
    pub_.publish(total_faces);
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
