#include "includes.ihh"
#include "noos_bridge.hpp"
#include "options.hpp"
#include "face_detector.hpp"

int main(int argc, char **argv) {

    options opt(argc, argv);
    auto noos_plat = opt.read();

    ros::init(argc, argv, "face_detection");
    ros::NodeHandle n;
    face_detector face_obj(noos_plat, n);
    read_image reader(n, opt.get_topic());
    ros::Subscriber sub;

    if (!opt.get_topic().empty() ||
        opt.get_topic() != "none") {
        sub = n.subscribe<sensor_msgs::Image>(opt.get_topic(),
                                              1000,
                                              &read_image::take_image,
                                              &reader);
    }
   
    ros::Rate loop_rate(0.5);
    while (ros::ok()) {
        auto matrix = reader.get_image();
        if (!matrix.empty()) {
            face_obj.detect(matrix);
        }
        loop_rate.sleep(); 
    }
    return 0;
}
