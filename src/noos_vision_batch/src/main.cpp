#include "includes.ihh"
#include "options.hpp"
#include "vision_batch_wrapper.hpp"

int main(int argc, char **argv) {

    options opt(argc, argv);
    auto noos_plat = opt.read();

    ros::init(argc, argv, "vision_batch");
    ros::NodeHandle n;
    vision_batch_wrapper v_obj(noos_plat, n);
    ros::Subscriber sub;

    if (!opt.get_topic().empty() ||
        opt.get_topic() != "none") {
        sub = n.subscribe<noos_vision_batch::faces_cropped>(opt.get_topic(),
                                                            1000,
                                                            &vision_batch_wrapper::get_image,
                                                            &v_obj);
    }
    else {
        throw std::runtime_error("No topic added. The subscriber hasn't been created");
    }
   
    ros::Rate loop_rate(0.5);
    ros::spin();
    loop_rate.sleep(); 

    return 0;
}
