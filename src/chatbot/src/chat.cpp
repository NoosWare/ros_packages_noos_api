#include "chat.hpp"

chat::chat(noos::cloud::platform plat,
           ros::NodeHandle n)
: callab_(std::bind(&chat::callback, this, std::placeholders::_1),
          plat,
          "hello"),
  pub_(n.advertise<std_msgs::String>("chatter", 1000))
{}

void chat::read_sentence(const std_msgs::String::ConstPtr & phrase) 
{
    if (phrase) {
        std::cout << "Receive: " << phrase->data.c_str() << std::endl;
        callab_.object = noos::cloud::chatbot(phrase->data);
        callab_.send();
    }
    else {
        std::cout << "No data received" << std::endl;
    }
}

void chat::callback(std::string sentence)
{
    if (!sentence.empty()) {
        std::cout << "Reply: " << sentence << std::endl;
        std_msgs::String msg;
        msg.data = sentence;
        pub_.publish(msg);
    }
}
