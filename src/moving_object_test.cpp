#include "MOR/MovingObjectRemoval.h"

extern ros::Publisher pub,marker_pub;

boost::shared_ptr<MovingObjectRemoval> mor;

void moving_object_test(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
	clock_t begin_time = clock();

	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(*input, cloud);

	mor->pushRawCloudAndPose(cloud,odm->pose.pose);
	if(mor->filterCloud("filtered"))
	{
		pub.publish(mor->output);
	}

	cout<<"-----------------------------------------------------\n";
    cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<endl;
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);
  marker_pub = nh.advertise<visualization_msgs::Marker>("bbox", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/velodyne_points", 1);
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/camera/odom/sample", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  sync.registerCallback(boost::bind(&moving_object_test, _1, _2));

  mor.reset(new MovingObjectRemoval(3,2));

  ros::spin();
}