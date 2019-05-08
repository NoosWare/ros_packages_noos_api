#include "includes.ihh"
#include "noos_path_planning/path.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "noos_path_planning_client_test");
  if (argc != 5)
  {
    ROS_INFO("usage: noos_path_planning_client_test map_name config_file resolution robot_radious");
    return 1;
  }

  geometry_msgs::Pose2D start;
  start.x = 0;
  start.y = 0;
  start.theta = 0;

  geometry_msgs::Pose2D goal;
  goal.x = 0;
  goal.y = 1;
  goal.theta = 0;

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<noos_path_planning::path>("path");
  noos_path_planning::path srv;
  srv.request.platform = std::string(argv[2]);
  srv.request.map_name = argv[1];
  srv.request.resolution = atof(argv[3]);
  srv.request.robot_radius = atof(argv[4]);
  srv.request.start = start;
  srv.request.goal = goal;

  if (client.call(srv))
  {
    if (srv.response.result.size() != 0) {
        for (auto point : srv.response.result) {
            std::cout << point.x << " " << point.y << std::endl; 
        }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service path");
    return 1;
  }

  return 0;
}
