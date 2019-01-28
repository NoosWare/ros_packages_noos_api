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

geometry_msgs::Pose noos_to_ros_pose::operator()(noos::object::pose<float> pos)
{
    geometry_msgs::Pose result;	
	result.position.x = pos.coordinates.x;
	result.position.y = pos.coordinates.y;
	result.position.z = pos.coordinates.z;

 	auto q = euler_2_quaternion()(pos.angles);	

	result.orientation.x = q.x;
	result.orientation.y = q.y;
	result.orientation.z = q.z;
	result.orientation.w = q.w;

	return result;
}

noos::object::laser laser_to_noos::operator()(const sensor_msgs::LaserScan::ConstPtr & scan)
{
    noos::object::laser obs;
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
    return obs;
}
