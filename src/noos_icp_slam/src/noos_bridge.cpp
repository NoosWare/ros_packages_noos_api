#include "noos_bridge.hpp"

quaternion euler_2_quaternion::operator()(noos::object::orientation<float> euler)
{
    quaternion q;

    // Abbreviations for the various angular functions
    double cy = cos(euler.yaw * 0.5);
    double sy = sin(euler.yaw * 0.5);
    double cp = cos(euler.pitch * 0.5);
    double sp = sin(euler.pitch * 0.5);
    double cr = cos(euler.roll * 0.5);
    double sr = sin(euler.roll * 0.5);

    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
    return q;
}

icp_slam::pose3d noos_to_ros_pose::operator()(noos::object::pose<float> pos,
											  std::string frame_id)
{
	icp_slam::pose3d msg;

	std_msgs::Header header;
	header.frame_id = frame_id;
    geometry_msgs::Pose result;	
	result.position.x = pos.coordinates.x;
	result.position.y = pos.coordinates.y;
	result.position.z = pos.coordinates.z;

 	auto q = euler_2_quaternion()(pos.angles);	

	result.orientation.x = q.x;
	result.orientation.y = q.y;
	result.orientation.z = q.z;
	result.orientation.w = q.w;

	msg.pose = result;
	msg.header = header;

	return msg;
}

icp_slam::pose2d noos_to_ros_pose2d::operator()(noos::object::pose<float> pos,
												std::string frame_id)
{
	icp_slam::pose2d msg;

	std_msgs::Header header;
	header.frame_id = frame_id;
    geometry_msgs::Pose2D result;	
	result.x = pos.coordinates.x;
	result.y = pos.coordinates.y;
    result.theta = pos.angles.yaw;
	
	msg.pose = result;
	msg.header = header;

	return msg;
}

noos::object::laser laser_to_noos::operator()(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    noos::object::laser obs;
    // 
    // All the parameters of laser MUST to be filled
    // For more information @see noos::object::laser
    //
    assert(scan);
    if (scan->ranges.size() != scan->intensities.size()) 
        throw std::runtime_error("Bad Laser data. Intensities and Ranges have different sizes");
    
    auto now = std::chrono::system_clock::now();
    obs.timestamp = now.time_since_epoch().count();
    obs.ranges.resize(scan->ranges.size());
    obs.intensities.resize(scan->intensities.size());
    obs.right_to_left = true;
    obs.aperture = scan->angle_max - scan->angle_min;
    obs.max_range = scan->range_max;
    obs.std_error = 0.020f;
    obs.pose3d = noos::object::pose<float>();

    for (size_t i = 0; i < scan->ranges.size(); i++) {
        obs.ranges[i] = scan->ranges[i];
        obs.intensities[i] = (int)scan->intensities[i];
    }

    return obs;
}
