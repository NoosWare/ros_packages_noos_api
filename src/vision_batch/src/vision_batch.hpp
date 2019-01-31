#ifndef VISION_BATCH_HPP
#define VISION_BATCH_HPP

#include "noos_bridge.hpp"
#include "includes.ihh"

using namespace noos::cloud;

/**
 * @class vision_batch
 * @brief detect age, gender and expression of a face
 * @version 0.1.0
 * @date 31.01.2019
 */
class vision_batch
{
public:
    ///@brief constructor
    vision_batch(noos::cloud::platform plat,
                 ros::NodeHandle node);

    ///@brief call the service detect faces
    void get_image(const vision_batch::faces_cropped & faces);

private:
    using vbatch = vision_batch<tied<face_expression>,tied<age_detection>>;

    //face expression callback
    void face_expression_cb(std::vector<std::pair<std::string,float>> data);

    //age detection callback
    void age_detection_cb(std::vector<std::pair<std::string,float>> data);

    //age detection callback
    void gender_detection_cb(std::vector<std::pair<std::string,float>> data);

    //send vision batch object
    void batch_send(noos::object::picture new_pic);

    // callable for vision_batch
    std::unique_ptr<callable<vbatch, true>> batch_;
    
    noos::cloud::tied<face_expression> exp_tie_;
	noos::cloud::tied<age_detection> age_tie_;
	noos::cloud::tied<gender_detection> gender_tie_;

    //Publisher
    ros::Publisher pub_;
    //Batch data
    batch_data all_data_;
};

#endif
