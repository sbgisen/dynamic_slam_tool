#include "MOR/IncludeAll.h"
#include "component_clustering.h"
#include "DBSCAN_clustering.h"
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy; //for message synchronization

//'xyz' -> refers to the variable with name xyz
//shared pointers are used for autometic garbage collection共享指针用于自动垃圾收集


struct MovingObjectDetectionCloud
{
	/*
	A structure for holding incoming pointcloud and corresponding odometry. It has functions for 
	pointcloud pre-processing, ground plane removal and euclidian clustering. It aslo stores the
	moving object detection results after computation.一种用于容纳输入点云和相应里程计的结构。
	它具有点云预处理、地平面去除和欧几里得聚类等功能。它还存储计算后的运动目标检测结果。
	*/
	float gp_limit,gp_leaf,bin_gap;
	long min_cluster_size,max_cluster_size;
	/*configuration variables配置变量*/
    

	pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud,cloud,cluster_collection;
	/*raw_cloud: stores the pointcloud after trimming it in x,y,z axis
	cloud: stores pointcloud after ground plane removal
	cluster_collection: stores pointcloud with all the detected clusters*/
    
	sensor_msgs::PointCloud2 output_rgp;
	/* 添加 */

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	/*vector to store the individual detected clusters
	用于存储单个检测到的群集的向量*/

	pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_collection;
	/*stores the centroid of all the detected clusters. indexing is same as 'clusters'
	存储所有检测到的簇的质心。索引与“clusters”相同*/

	std::vector<pcl::PointIndices> cluster_indices;
	
	/*vector to store the indices of all the detected clusters in 'cloud'
	将所有检测到的群集的索引存储在“云”中的向量*/

	pcl::IndicesConstPtr gp_indices;
	/*indices of the points of ground plane in 'raw_cloud' which gets removed wile extracting 'cloud'
	“raw_cloud”中通过提取“cloud”删除的地平面点的索引*/

	std::vector<bool> detection_results; 
	/*results of moving object detection. indexing is same as 'clusters'
	运动目标检测结果。索引与“clusters”相同*/

	tf::Pose ps;
	/*stores the 6D pose at which the pointloud was captured.
	存储捕捉pointloud时的6D姿势*/
	
	bool init;//helps in synchronization有助于同步

	MovingObjectDetectionCloud(float gp_lm,float gp_lf,float bin_g,long min_cl_s,long max_cl_s)
	:gp_limit(gp_lm),gp_leaf(gp_lf),bin_gap(bin_g),min_cluster_size(min_cl_s),max_cluster_size(max_cl_s)
	//初始化类成员（成员初始化列表），没有该参数无法创建一个类对象
	{
		//constructor to initialize shared pointers and default variables
		//初始化共享指针和默认变量的构造函数
		raw_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
		cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
		cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
		centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
		init = false;
	}
	void groundPlaneRemoval(float,float,float); //ground plane removal with predefined dimensions
	void groundPlaneRemoval(float,float); //ground plane removal using voxel covariance and binning
	void computeClusters(float,std::string); //computes the euclidian clustering with an input distance threshold

	
};

class MovingObjectDetectionMethods
{
	/*
	A class to implement the methods and constraints for moving object detection between consecutive
	pointclouds.用于实现连续点云之间移动对象检测的方法和约束的类。
	*/

	float volume_constraint,pde_lb,pde_ub;

	public:
		MovingObjectDetectionMethods(float v_c,float p_l,float p_u):volume_constraint(v_c),pde_lb(p_l),pde_ub(p_u){}

		bool volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold);
		/*checks if two corresponding clusters have nearly equal volume, using an input threshold
		使用输入阈值检查两个相应群集的体积是否几乎相等*/

		void calculateCorrespondenceCentroid(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr mp,double delta);
		/*cluster centroid reciprocal correspondence based on euclidian distance*/

		std::vector<double> getPointDistanceEstimateVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp);
		/*implimentation of point correspondence distance estimation approach*/

		std::vector<double> getClusterPointcloudChangeVector(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,pcl::CorrespondencesPtr mp,float resolution);
		/*implementation of octree pointcloud change approach*/
};

//一种存储移动簇质心属性的结构
struct MovingObjectCentroid
{
	/*
	A structure for storing the properties of a moving cluster centroid.
	一种存储移动簇质心属性的结构*/
	pcl::PointXYZ centroid; //centroid of moving cluster
	int confidence,max_confidence; //moving confidence score

	MovingObjectCentroid(pcl::PointXYZ c,int n_good):centroid(c),confidence(n_good+1),max_confidence(n_good+1){} 
	//constructor构造
	bool decreaseConfidence(){confidence--;if(confidence==0){return true;}return false;} 
	//returns true when confidence reduces to 0
	void increaseConfidence(){if(confidence<max_confidence){confidence++;}} 
	//increases confidence till 'max_confidence'
};

class MovingObjectRemoval
{
	/*
	A class for implementing the algorithms for detection and removal of moving objects. It includes
	cluster tracking using confidence scores and handles the input data in a serial in and serial out manner. 
	用于实现运动对象检测和移除算法的类。它包括使用置信度得分的群集跟踪，并以串行输入和串行输出的方式
	处理输入数据*/

	float gp_limit,gp_leaf,bin_gap,volume_constraint,pde_lb,pde_ub,leave_off_distance,catch_up_distance,trim_x,trim_y,trim_z,ec_distance_threshold,pde_distance_threshold;
	long min_cluster_size,max_cluster_size;
	int method_choice,opc_normalization_factor;
	std::string output_topic,debug_topic,marker_topic,input_pointcloud_topic,input_odometry_topic,output_fid,debug_fid;
	/*configuration variables*/

	std::vector<MovingObjectCentroid> mo_vec;
	/*vector to store the detected and confirmed moving cluster properties
	用于存储检测到和确认的移动群集属性的向量*/

	std::deque<pcl::CorrespondencesPtr> corrs_vec;
	/*double ended queue to store the consequtive frame cluster correspondece results as a buffer
	用于将Consequitive frame群集对应结果存储为缓冲区的双端队列*/

	std::deque<std::vector<bool>> res_vec;
	/*double ended queue to store the frame moving object detection results as a buffer
	用于将帧移动对象检测结果存储为缓冲区的双端队列*/

	/*deque is an optimized DS optimized for deletion at both begenning and end. As 'corrs_vec' and
	'res_vec' are buffers they are better stored as deques deque是一个优化的DS，优化后可在开始和
	结束时删除。由于“corrs_vec”和“res_vec”是缓冲区，因此最好将它们存储为deque*/

	boost::shared_ptr<MovingObjectDetectionCloud> ca,cb;
	/*shared pointers for holding incoming data用于保存传入数据的共享指针*/

	boost::shared_ptr<MovingObjectDetectionMethods> mth;
	/*shared pointers for detection class object检测类对象的共享指针*/

	int moving_confidence,static_confidence;
	/*confidence score for detection聚类的置信度得分*/

	pcl::KdTreeFLANN<pcl::PointXYZ> xyz_tree;
	/*search tree for matching moving cluster centroids with latest cluster centroid collection
	用于将移动的群集质心与最新的群集质心集合匹配的搜索树*/

	ros::NodeHandle& nh; //for visualization and internal sync
	#ifdef VISUALIZE
	ros::Publisher pub,debug_pub,marker_pub,marker1_pub,marker2_pub,marker3_pub,marker4_pub,gpr; //for visualization
	#endif
	#ifdef INTERNAL_SYNC //internal synchronization implementation
	message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub;
	boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;
	#endif

	void setVariables(std::string config_file_path);
	/*sets the algorithm variables from the config file*/

	void movingCloudObjectSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm);
	/*call back to get incoming data using internal sync*/

	int recurseFindClusterChain(int col,int track);
	/*recursive function to check for moving cluster chain consistency*/

	void checkMovingClusterChain(pcl::CorrespondencesPtr mp,std::vector<bool> &res_ca,std::vector<bool> &res_cb);
	/*function to confirm the consistency of a moving cluster and add the cluster to the 'mo_vec'*/

	void pushCentroid(pcl::PointXYZ pt);
	/*function to add the centroid of a new moving cluster to 'mo_vec'*/
	int compare(int a,std::vector<int>b);
	
	public:
		sensor_msgs::PointCloud2 output; //stores the pointcloud after the moving objects removal
		//删除移动对象后存储点云
	
		MovingObjectRemoval(ros::NodeHandle _nh,std::string config_path,int n_bad,int n_good);
		/*constructor: config_path is the path to the configuration file for the package*/

		void pushRawCloudAndPose(pcl::PCLPointCloud2 &cloud,geometry_msgs::Pose pose);
		/*Input: function to push new data into the running alogorithm*/

		void showDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &c2);

		void showV(pcl::PointCloud<pcl::PointXYZ>::Ptr &c1,pcl::PointCloud<pcl::PointXYZ>::Ptr &c2,
		pcl::CorrespondencesPtr fmp,std::vector<bool> &res_cb);

		bool filterCloud(pcl::PCLPointCloud2 &cloud,std::string f_id);
		/*Output: function to get the filtered cloud after moving object removal*/
};