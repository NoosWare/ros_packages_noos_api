#ifndef SLAM_HPP
#define SLAM_HPP
#include "includes.ihh"
#include "noos_bridge.hpp"

//using namespace noos::cloud;

/**
 * @brief Send laser data to the Noos Cloud and
 *        publish the position of the robot
 * @class slam
 * @date 25.01.2019
 * @version 0.1.0
 * @author Maria Ramos
 */
class slam
{
public:
    /// @brief constructor
    slam(noos::cloud::platform plat,
         ros::NodeHandle & n);

    /// @brief read laser data
    void read_laser(const sensor_msgs::LaserScan::ConstPtr & scan);

    /// @brief the data is sent to the platform
    void process_data(noos::object::laser & obs);

private:

    //Callback
    void callback(noos::object::pose<float> pose3d);
    //Callable object
    noos::cloud::callable<noos::cloud::icp_slam, true> callab_;
    //Count time between calls
    double t_begin_;
    double t_savemap_;
    //ros publisher
    ros::Publisher pub_;

};


/**
 * @brief Send icp configuration file to allow slam
 * @class send_icp_file
 * @date 25.01.2019
 * @version 0.1.0
 * @author Maria Ramos
 */
class send_icp_file
{
public:
    /// @brief constructor
    send_icp_file(noos::cloud::platform plat,
                  std::string config_file);

private:
    //callback required
    void config_callback(bool success);
    //configuration file
    noos::object::config_file config_;
    //callable object
    noos::cloud::callable<noos::cloud::upload_slam_config_file> config_callable_;
};

#endif
