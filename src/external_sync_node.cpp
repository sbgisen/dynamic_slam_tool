#include "MOR/MovingObjectRemoval.h"
// #define VISUALIZE
ros::Publisher pub;

boost::shared_ptr<MovingObjectRemoval> mor;

void moving_object_test(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
  clock_t begin_time = clock();
  // std::cout << "-----------------------------------------------------\n";
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*input, cloud);

  mor->pushRawCloudAndPose(cloud, odm->pose.pose);
  if (mor->filterCloud(cloud, input->header.frame_id))
  {
    pub.publish(mor->output);
  }

  // std::cout << 1000.0 * (clock() - begin_time) / CLOCKS_PER_SEC << std::endl;
  // std::cout << "-----------------------------------------------------\n";
}

int main(int argc, char** argv)
{
#ifndef INTERNAL_SYNC
  ros::init(argc, argv, "test_moving_object");
  ros::NodeHandle nh("~");

  std::string output_topic, input_pointcloud_topic, input_odometry_topic;
  nh.param<std::string>("output_topic", output_topic, "/velodyne/points_static");
  nh.param<std::string>("input_pointcloud_topic", input_pointcloud_topic, "/velodyne_points");
  nh.param<std::string>("input_odometry_topic", input_odometry_topic, "/odom");

  pub = nh.advertise<sensor_msgs::PointCloud2>(output_topic, 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, input_pointcloud_topic, 5);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, input_odometry_topic, 5);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  sync.registerCallback(boost::bind(&moving_object_test, _1, _2));

  mor.reset(new MovingObjectRemoval(nh, 4, 3));

  ros::spin();
#endif
}
