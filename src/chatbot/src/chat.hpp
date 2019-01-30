#ifndef CHAT_HPP
#define CHAT_HPP

#include "includes.ihh"

/**
 * @class chat
 * @brief class for using noos chat service
 * @version 0.1.0
 * @date 29.01.2019
 */
class chat
{
public:
    ///@brief Constructor using platform data and ros node
    chat (noos::cloud::platform plat,
          ros::NodeHandle n);

    ///@brief read the sentence sent by user
    void read_sentence(const std_msgs::String::ConstPtr & phrase);

private:
    //Callback
    void callback(std::string sentence);
    //Callable object
    noos::cloud::callable<noos::cloud::chatbot, true> callab_;
    //ros publisher
    ros::Publisher pub_;
};

#endif
