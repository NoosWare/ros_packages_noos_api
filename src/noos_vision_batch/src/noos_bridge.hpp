#ifndef NOOS_BRIDGE_HPP
#define NOOS_BRIDGE_HPP

#include "includes.ihh"

/**
 * @struct batch_data
 * @brief struct to save the age,
 *        gender and expression of a face
 * @version 0.1.0
 * @date 31.01.2019
 */
struct batch_data
{
    std::vector<std::pair<std::string,float>> age;
    std::vector<std::pair<std::string,float>> gender;
    std::vector<std::pair<std::string,float>> expression;

    ///@brief clean all data
    void clean();

    ///@brief check all vectors have been filled
    bool check_all_filled();
};

/**
 * @struct mat_to_picture
 * @brief Doing the conversion from cv::Mat to noos::object::picture
 * is not trivial. With this struct the conversion is done
 * for .png images. If another extension is needed 
 * @version 0.1.0
 * @date 29.01.2019
 */
struct mat_to_picture
{
    noos::object::picture operator()(cv::Mat img);
};

/**
 * @struct mat2ros_image
 * @brief convert cv::Mat to sensor_msgs::Image
 * @date 31.01.2019
 * @version 0.1.0
 */
struct mat2ros_image
{
    sensor_msgs::Image operator()(cv::Mat img,
                                  int counter);
};

/**
 * @struct data2pair_vector
 * @brief convert std::vector<std::pair<std::string, float>>
 *        into pair_vector data (custom msg)
 * @date 31.01.2019
 * @version 0.1.0
 */
struct data2pair_vector
{
    noos_vision_batch::pair_vector operator()(std::vector<std::pair<std::string, float>> data);
};

/**
 * @struct pair_vector2batch
 * @brief  pair_vector data (custom msg) into batch data (custom msg)
 * @date 31.01.2019
 * @version 0.1.0
 */
struct pair_vector2batch
{
    noos_vision_batch::batch operator()(int face_number, 
                                        noos_vision_batch::pair_vector age,
                                        noos_vision_batch::pair_vector gender,
                                        noos_vision_batch::pair_vector expression);
};

#endif
