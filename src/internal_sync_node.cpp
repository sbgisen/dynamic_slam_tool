#include "MOR/MovingObjectRemoval.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_moving_object");
  ros::NodeHandle nh("~");
  MovingObjectRemoval mor(nh, 4, 3);
  ros::spin();
}
