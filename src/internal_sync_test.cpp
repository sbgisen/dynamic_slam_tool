#include "MOR/MovingObjectRemoval.h"

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;

  MovingObjectRemoval mor(nh,"/home/wentaoq/lidar_ws/src/dynamicslamtool-master3/config/MOR_config.txt",4,3);

  ros::spin();
}