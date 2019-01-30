#include "includes.ihh"
#include "chat.hpp"
#include "options.hpp"

int main(int argc, char **argv)
{
    options opt(argc, argv);
    auto noos_plat = opt.read();

    ros::init(argc, argv, "chat");
    ros::NodeHandle n;

    ros::Rate loop_rate(1);
    chat chat_obj(noos_plat, n);

    ros::Subscriber sub = n.subscribe<std_msgs::String>("talker", 1000, &chat::read_sentence, &chat_obj); 

    ros::spin();
    loop_rate.sleep();

    return 0;
}
