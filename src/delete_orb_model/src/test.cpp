#include "includes.ihh"
#include "delete_orb_model/orb_delete.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "delete_orb_model_client_test");
  if (argc != 3)
  {
    ROS_INFO("usage: delete_orb_model_client_test model config_file");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<delete_orb_model::orb_delete>("orb_delete");
  delete_orb_model::orb_delete srv;
  srv.request.platform = std::string(argv[2]);
  srv.request.model = argv[1];

  if (client.call(srv))
  {
    ROS_INFO("Model deleted?: %d", (bool)srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service orb_delete");
    return 1;
  }

  return 0;
}
