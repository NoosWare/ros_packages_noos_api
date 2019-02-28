#ifndef NOOS_BRIDGE_HPP
#define NOOS_BRIDGE_HPP

#include "includes.ihh"

#define M_PIf 3.14159265358979f

/**
 * @struct quaternion
 * @version 0.1.0
 */
struct quaternion
{
    double x = 0;
    double y = 0;
    double z = 0;
    double w = 0;
};

/**
 * @struct euler_2_quaternion
 * @brief convert euler angles to quaternion
 * @version 0.1.0
 * @date 28.01.2019
 */
struct euler_2_quaternion
{
    quaternion operator()(noos::object::orientation<float> euler);
};

/**
 * @struct noos_to_ros_pose
 * @brief convert noos::object::pose3d to ros geometry_msgs::Pose
 * @version 0.1.0
 * @date 28.01.2019
 */
struct noos_to_ros_pose
{
    geometry_msgs::Pose operator()(noos::object::pose<float> pos);
};

/**
 * @struct noos_to_ros_pose2d
 * @brief convert noos::object::pose3d to ros geometry_msgs::Pose2D
 * @version 0.1.0
 * @date 28.02.2019
 */
struct noos_to_ros_pose2d
{
    geometry_msgs::Pose2D operator()(noos::object::pose<float> pos);
};

/**
 * @struct laser_to_noos
 * @brief convert sensor_msgs::LaserScan::ConstPtr to noos::object::laser
 * @version 0.1.0
 * @date 28.01.2019
 */
struct laser_to_noos
{
    noos::object::laser operator()(const sensor_msgs::LaserScan::ConstPtr & scan);
};

#endif
