#include "slam.hpp"

slam::slam(noos::cloud::platform plat,
           ros::NodeHandle & n,
           icp_args args) 
: callab_( std::bind(&slam::callback, this, std::placeholders::_1),
           plat,
           args.map_name, "icp.ini", noos::object::laser()),
  t_begin_(ros::Time::now().toSec()),
  pub_(n.advertise<icp_slam::pose3d>("pose", 1000)),
  pub_2d_(n.advertise<icp_slam::pose2d>("pose2d", 1000)),
  map_(plat, args.map_name),
  robot_name_(args.robot_name),
  map_name_(args.map_name)
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
        if (scan) { 
            auto obs = laser_to_noos()(scan);    
            //Now the laser object is complete, its data can be 
            //sent to the platform.
            process_data(obs);
            map_.get_map();
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
    callab_.object = noos::cloud::icp_slam(map_name_, "icp.ini", obs); //map, config, laser
    callab_.send();
}

void slam::callback(noos::object::pose<float> pose3d)
{
    //
    //The position of the robot in the map is showed
    //
    //std::cout << pose3d;
    pub_.publish(noos_to_ros_pose()(pose3d, robot_name_));
    pub_2d_.publish(noos_to_ros_pose2d()(pose3d, robot_name_));
    //
    //The time is reset to wait another 100 ms for the next call
    //
    t_begin_ = ros::Time::now().toSec();
}

send_icp_file::send_icp_file(noos::cloud::platform plat,
                             std::string config_file)
: config_(config_file),    
  config_callable_(std::bind(&send_icp_file::config_callback, this, std::placeholders::_1), 
                     plat, 
                     config_, 
                     "icp.ini", 
                     noos::cloud::slam_type::icp)
{
    config_callable_.send();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void send_icp_file::config_callback(bool success)
{
    if (!success) {
        throw std::runtime_error("Error uploading config file to the cloud");
    }
    std::cout << std::boolalpha << success << std::endl;
}

receive_map::receive_map(noos::cloud::platform plat,
                         std::string map_name)
: callab_(std::bind(&receive_map::callback, this, std::placeholders::_1),
          plat,
          map_name),
  t_savemap_(ros::Time::now().toSec())
{}

void receive_map::callback(bool success)
{
    if (success) {
        std::cout << "Map downloaded correctly" << std::endl;
    }
    else {
        std::cout << "Map couldn't be downloaded" << std::endl;
    }
}

void receive_map::get_map()
{
    if (ros::Time::now().toSec() - t_savemap_ > 20) {
        callab_.send();
        t_savemap_ = ros::Time::now().toSec();
    }
}
