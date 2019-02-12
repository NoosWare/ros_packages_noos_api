#include "includes.ihh"
#include "noos_delete_map/map_delete.h"

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

bool delete_slam_map(noos_delete_map::map_delete::Request & req,
                     noos_delete_map::map_delete::Response & res)
{
    auto plat = read_file(req.platform);
    noos::cloud::callable<noos::cloud::delete_map, 
                         false> call([&](bool result){
                                         std::cout << "cleared: " << result << std::endl;
                                         res.result = result;
                                     }, 
                                     plat, 
                                     req.map_name);
    call.send();
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noos_delete_map");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("map_delete", delete_slam_map);
  ROS_INFO("Ready to delete a map if it is required.");
  ros::spin();

  return 0;
}

