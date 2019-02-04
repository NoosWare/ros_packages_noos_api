#ifndef VISION_BATCH_WRAPPER_HPP
#define VISION_BATCH_WRAPPER_HPP

#include "noos_bridge.hpp"
#include "includes.ihh"

/**
 * @class vision_batch
 * @brief detect age, gender and expression of a face
 * @version 0.1.0
 * @date 31.01.2019
 */
class vision_batch_wrapper
{
public:
    ///@brief constructor
    vision_batch_wrapper(noos::cloud::platform plat,
                         ros::NodeHandle node);

    ///@brief call the service detect faces
    void get_image(noos_vision_batch::faces_cropped msg);

private:
    using vbatch = noos::cloud::vision_batch<noos::cloud::tied<noos::cloud::face_expression>,
                                             noos::cloud::tied<noos::cloud::age_detection>,
                                             noos::cloud::tied<noos::cloud::gender_detection>>;

    //send vision_batch information to the cloud
    void send_info(noos::object::picture image);

    //face expression callback
    void face_expression_cb(std::vector<std::pair<std::string,float>> data);

    //age detection callback
    void age_detection_cb(std::vector<std::pair<std::string,float>> data);

    //age detection callback
    void gender_detection_cb(std::vector<std::pair<std::string,float>> data);

    //send vision batch object
    void publish_result();

    // callable for vision_batch
    std::unique_ptr<noos::cloud::callable<vbatch, true>> batch_;
    
    noos::cloud::tied<noos::cloud::face_expression> exp_tie_;
	noos::cloud::tied<noos::cloud::age_detection> age_tie_;
	noos::cloud::tied<noos::cloud::gender_detection> gender_tie_;

    //Platform
    noos::cloud::platform plat_;
    //Publisher
    ros::Publisher pub_;
    //Batch data
    batch_data all_data_;
    //counter
    int counter = 0;
    //bool 
    bool finished = false;
};

#endif
