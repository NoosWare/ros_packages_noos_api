#include "slam.hpp"

slam::slam() 
: callab_( std::bind(&slam::callback, this, std::placeholders::_1),
        {"10.130.3.21", "9001", "test_token", "test"},
           "icp_map", "icp.ini", noos::object::laser()),
  t_begin_(ros::Time::now().toSec())
{}

void slam::read_laser(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    //
    //The frequency of the laser is higer than the processing time in 
    //the platform (40-50 ms). So the measures are going to be sent every
    //~100 ms.
    //
    if (ros::Time::now().toSec() - t_begin_ > 0.1) {
        assert(scan);
        noos::object::laser obs;
        if (scan) { 
            // 
            // All the parameters of laser MUST to be filled
            // For more information @see noos::object::laser
            //
            int count = scan->scan_time / scan->time_increment;
            auto now = std::chrono::system_clock::now();
            obs.timestamp = now.time_since_epoch().count();
            obs.ranges.resize(count);
            obs.intensities.resize(count);
            obs.right_to_left = false;
            obs.aperture = 2 * M_PIf;
            obs.max_range = 6.0;
            obs.std_error = 0.010f;
            obs.pose3d = noos::object::pose<float>();

            for (int i = 0; i < count; i++) {
                obs.ranges[i] = scan->ranges[i];
                obs.intensities[i] = (int)scan->intensities[i];
            }
            //
            //Now the laser object is complete, its data can be 
            //sent to the platform.
            process_data(obs);
        }
        else {
            std::cout << "No laser data" << std::endl;
        }
    }
}

void slam::process_data(noos::object::laser & obs)
{
    //
    //The object of the class callable is updated with the new laser data.
    //
    callab_.object = noos::cloud::icp_slam("icp_map", "icp.ini", obs); //map, config, laser
    callab_.send();
}

void slam::callback(noos::object::pose<float> pose3d)
{
    //
    //The position of the robot in the map is showed
    //
    std::cout << pose3d;
    //
    //The time is reset to wait another 100 ms for the next call
    //
    t_begin_ = ros::Time::now().toSec();
}

send_icp_file::send_icp_file(platform plat,
                             std::string config_file)
: config_(config_file),    
  config_callable_(std::bind(&send_icp_file::config_callback, this, std::placeholders::_1), 
                     plat, 
                     config_, 
                     "icp.ini", 
                     slam_type::icp)
{
    config_callable_.send();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void send_icp_file::config_callback(bool success)
{
    if (!success) {
        throw std::runtime_error("Error uploading config file to the cloud");
    }
}
