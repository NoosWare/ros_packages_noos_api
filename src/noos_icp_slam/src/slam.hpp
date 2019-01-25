#ifndef SLAM_HPP
#define SLAM_HPP
#include "includes.ihh"

#define M_PIf 3.14159265358979f

using namespace noos::cloud;
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
    slam();

    /// @brief read laser data
    void read_laser(const sensor_msgs::LaserScan::ConstPtr & scan);

    /// @brief the data is sent to the platform
    void process_data(noos::object::laser & obs);

private:

    //Callback
    void callback(noos::object::pose<float> pose3d);
    //Callable object
    callable<icp_slam, true> callab_;
    //Count time between calls
    double t_begin_;
    double t_savemap_;

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
    send_icp_file(platform plat,
                  std::string config_file);

private:
    //callback required
    void config_callback(bool success);

    //callable object
    callable<upload_slam_config_file> config_callable_;
    //platform
    platform node_;
    //configuration file
    noos::object::config_file config_;

};

#endif
