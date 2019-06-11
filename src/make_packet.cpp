#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <test_work/pc_odom.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>

using namespace std;

ros::Publisher pub;

void pack_data(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
  test_work::pc_odom msg;
  msg.header.stamp = ros::Time((input->header.stamp.toSec()+odm->header.stamp.toSec())/2);
  cout<<input->header.stamp<<" "<<odm->header.stamp<<" "<<msg.header.stamp<<endl;
  msg.cloud = *input;
  msg.odom = *odm;

  pub.publish(msg);
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "packing_node");
  ros::NodeHandle nh;

  pub = nh.advertise<test_work::pc_odom>("/sync_packet", 50);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/camera/odom/sample", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  sync.registerCallback(boost::bind(&pack_data, _1, _2));

  ros::spin();
}
