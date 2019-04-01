#include "includes.ihh"
#include "options.hpp"
#include "slam.hpp"


int main(int argc, char **argv)
{
    /* First send the config file */
    options opt(argc, argv);
    auto noos_plat = opt.read();
    auto arguments = opt.get_icp_data();
    if (!arguments.loaded) {
        send_icp_file send_file(noos_plat, arguments.config_file);
    }
	
	/* SLAM*/
	ros::init(argc, argv, "icp_slam");
	ros::NodeHandle n;

	ros::Rate loop_rate(1);
    slam slam_obj(noos_plat, n, arguments);

	// "/scan" is for Laser messages
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>(arguments.topic, 1000, &slam::read_laser, &slam_obj);

	ros::spin();
	loop_rate.sleep(); 

    return 0;
}
