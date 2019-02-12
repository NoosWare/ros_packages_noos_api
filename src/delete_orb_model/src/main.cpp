#include "includes.ihh"
#include "delete_orb_model/orb_delete.h"

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

bool delete_model(delete_orb_model::orb_delete::Request & req,
                  delete_orb_model::orb_delete::Response & res)
{
    auto plat = read_file(req.platform);
    noos::cloud::callable<noos::cloud::orb_del_model, 
                         false> call([&](bool result){
                                         std::cout << "cleared: " << result << std::endl;
                                         res.result = result;
                                     }, 
                                     plat, 
                                     req.model);
    call.send();
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "delete_orb_model");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("orb_delete", delete_model);
  ROS_INFO("Ready to delete an orb model if it is required.");
  ros::spin();

  return 0;
}

