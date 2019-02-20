#ifndef COMMON_HPP
#define COMMON_HPP

#include "includes.ihh"

/**
 * @struct noos_point2ros
 * @brief convert noos::object::point2d into 
 *        geometry_msgs/Point
 * @version 0.1.0
 */
struct noos_point2ros
{
    geometry_msgs::Point operator()(noos::object::point2d<float> noos_point) {
        geometry_msgs::Point ros_point;
        ros_point.x = noos_point.x;
        ros_point.y = noos_point.y;
        ros_point.z = 0;
        return ros_point;
    };
};

/**
 * @struct ros_point2noos
 * @brief convert geometry_msgs/Pose2D into 
 *        noos::object::pose2d
 * @version 0.1.0
 */
struct ros_pose2noos
{
     noos::object::pose2d<float> operator()(geometry_msgs::Pose2D ros_pose) {
        noos::object::pose2d<float> noos_pose;
        noos_pose.x = ros_pose.x;
        noos_pose.y = ros_pose.y;
        noos_pose.theta = ros_pose.theta;
        return noos_pose;
    };
};

#endif
