#include "MOR/MovingObjectRemoval.h"
// #define VISUALIZE
ros::Publisher pub;

boost::shared_ptr<MovingObjectRemoval> mor;

void moving_object_test(const sensor_msgs::PointCloud2ConstPtr& input)
{
	clock_t begin_time = clock();
  std::cout<<"-----------------------------------------------------\n";
	pcl::PCLPointCloud2 cloud;
	pcl_conversions::toPCL(*input, cloud);

   geometry_msgs::Pose  le;
        le.position.x = 0;
        le.position.y = 0;
        le.position.z = 0;
        le.orientation.x = 0;
        le.orientation.y = 0;
        le.orientation.z = 0;
        le.orientation.w = 1;
  
	mor->pushRawCloudAndPose(cloud,le);
	if(mor->filterCloud(cloud,"velodyne"))
	{
		pub.publish(mor->output);
	}

  std::cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<std::endl;

  std::cout<<"-----------------------------------------------------\n";
}

int main (int argc, char** argv)
{
  #ifndef INTERNAL_SYNC 
  ros::init (argc, argv, "test_moving_object");
  ros::NodeHandle nh;
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 10);

  ros::Subscriber pc_sub=nh.subscribe( "/velodyne_points", 1,moving_object_test);
  //message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/integrated_imu_test", 1);
  //ros::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/integrated_odom", 1);
  //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
  //message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
  //sync.registerCallback(boost::bind(&moving_object_test, _1, _2));

  mor.reset(new MovingObjectRemoval(nh,"/home/wyw/catkin_ws/src/dynamicslamtool-master3/config/MOR_config.txt",4,3));

  ros::spin();
  #endif
}