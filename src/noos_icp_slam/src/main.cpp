#include "includes.ihh"
#include "options.hpp"
#include "slam.hpp"


int main(int argc, char **argv)
{
	using namespace noos::cloud;
    /* First send the config file */

	/* SLAM*/
	ros::init(argc, argv, "icp_slam");
	ros::NodeHandle n;

	ros::Rate loop_rate(1);
	slam slam_obj;

	// "/scan" is for RPLIDAR messages
	ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &slam::read_laser, &slam_obj);

	ros::spin();
	loop_rate.sleep(); 

    return 0;
}
