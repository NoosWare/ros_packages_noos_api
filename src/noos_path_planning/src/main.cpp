#include "includes.ihh"
#include "common.hpp"
#include "noos_path_planning/path.h"

noos::cloud::platform read_file(std::string config_file)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(config_file, pt);

    noos::cloud::platform node;
    node.address = pt.get("platform.address", "demo.noos.cloud");
    node.port = pt.get("platform.port", "9001");
    node.user = pt.get<std::string>("platform.user");
    node.token = pt.get<std::string>("platform.pass");
    
    return node;
}

bool get_path(noos_path_planning::path::Request & req,
              noos_path_planning::path::Response & res)
{
    auto plat = read_file(req.platform);
    noos::cloud::callable<noos::cloud::path_planning, 
                         false> call([&](std::deque<noos::object::point2d<float>> result){
                                         for (auto each_point : result) {
                                             res.result.push_back(noos_point2ros()(each_point));
                                         }
                                     }, 
                                     plat, 
                                     ros_pose2noos()(req.start),
                                     ros_pose2noos()(req.goal),
                                     req.robot_radius,
                                     req.resolution,
                                     req.map_name);
    call.send();
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noos_path_planning");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("path", get_path);
  ROS_INFO("Ready to calculate a path.");
  ros::spin();

  return 0;
}

