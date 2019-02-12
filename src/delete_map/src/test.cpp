#include "includes.ihh"
#include "noos_delete_map/map_delete.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noos_delete_map_client_test");
  if (argc != 3)
  {
    ROS_INFO("usage: noos_delete_map_client_test map_name config_file");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<noos_delete_map::map_delete>("map_delete");
  noos_delete_map::map_delete srv;
  srv.request.platform = std::string(argv[2]);
  srv.request.map_name= argv[1];

  if (client.call(srv))
  {
    ROS_INFO("Map deleted?: %d", (bool)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service map_delete");
    return 1;
  }

  return 0;
}
