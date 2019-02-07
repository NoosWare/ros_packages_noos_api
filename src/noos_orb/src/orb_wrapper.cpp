#include "orb_wrapper.hpp"

orb_wrapper::orb_wrapper(noos::cloud::platform plat,
                         ros::NodeHandle node,
                         orb_data data)
: pub_(node.advertise<noos_orb::point_array>("orb", 2000)),
  plat_(plat),
  orb_data_(data)
{
    if (!orb_data_.loaded) {
        upload_image(orb_data_.filename);    
    }
}

void orb_wrapper::detect(const cv::Mat & pict)
{
    if (pict.empty()) {
        return;
    }
    image_ = pict;
    auto pic = mat_to_picture()(pict);
    if (!pic.type().empty()) {
        noos::cloud::callable<noos::cloud::orb_query, 
                              false> call(std::bind(&orb_wrapper::callback, this, std::placeholders::_1),
                                          plat_,
                                          pic,
                                          orb_data_.filename,
                                          orb_data_.threshold);
        call.send();
    }
    else {
        std::cout << "error with image" << std::endl;
    }
}

void orb_wrapper::upload_image(std::string filename)
{
    auto matrix = cv::imread(filename);
    if (matrix.empty()) {
        throw std::runtime_error("No image in the file given");
    }
    auto pic = mat_to_picture()(matrix);
    noos::cloud::callable<noos::cloud::orb_add_model, 
                         false> call0(std::bind(&orb_wrapper::upload_cb, this, std::placeholders::_1),
                                      plat_,
                                      pic,
                                      filename);
    call0.send();
}

void orb_wrapper::callback(std::vector<noos::object::point2d<float>> points)
{
	pub_.publish(data2point_array()(points));
}

void orb_wrapper::upload_cb(bool result)
{
    if (!result) {
        throw std::runtime_error("Error uploading the image to the cloud");
    }
    std::cout << "Image uploaded to the cloud" << std::endl;
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
