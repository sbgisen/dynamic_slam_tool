#include "MOR/MovingObjectRemoval.h"

//'xyz' -> refers to the variable with name xyz

////////////////////////////////////////////////////////////////////Helping Methods

//函数生成边界框可视化
visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, 
int id, std::string f_id, std::string ns="bounding_box", float r=0.5, float g=0.5, float b=0.5)
{
  /*Function to generate bounding box visualization markers. This function is used when the VISUALIZE
  flag is defined*/
  Eigen::Vector4f centroid;
  Eigen::Vector4f min;
  Eigen::Vector4f max;

  pcl::compute3DCentroid(*cloud_cluster, centroid);
  pcl::getMinMax3D(*cloud_cluster, min, max);

  uint32_t shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  marker.header.frame_id = f_id;
  marker.header.stamp = ros::Time::now();

  marker.ns = ns;
  marker.id = id;
  marker.type = shape;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = centroid[0];
  marker.pose.position.y = centroid[1];
  marker.pose.position.z = centroid[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = (max[0]-min[0]);
  marker.scale.y = (max[1]-min[1]);
  marker.scale.z = (max[2]-min[2]);

  if (marker.scale.x ==0)
      marker.scale.x=0.1;

  if (marker.scale.y ==0)
    marker.scale.y=0.1;

  if (marker.scale.z ==0)
    marker.scale.z=0.1;

  //colour of the box
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.5; //opacity

  marker.lifetime = ros::Duration(2); //persistance duration
  //marker.lifetime = ros::Duration(10);
  return marker;
}

//函数生成边界框可视化
visualization_msgs::Marker mark_cluster2(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster, 
int id, std::string f_id,int colour){
  
  Eigen::Vector4f min;
  Eigen::Vector4f max;
  int numPoints = cloud_cluster->size(); 
  pcl::getMinMax3D(*cloud_cluster, min, max);
  vector<Point> pointVec(numPoints);
  for(int iPoint = 0; iPoint < cloud_cluster->size(); iPoint++){
    float pX = cloud_cluster->points[iPoint].x;
    float pY = cloud_cluster->points[iPoint].y;
    float pZ = cloud_cluster->points[iPoint].z;
    float roiX = pX + roiM/2;
    float roiY = pY + roiM/2;
    int x = floor(roiX*picScale);
    int y = floor(roiY*picScale);
    pointVec[iPoint] = Point(x, y); 
  }

  RotatedRect rectInfo = minAreaRect(pointVec);
  Point2f rectPoints[4]; 
  rectInfo.points(rectPoints);

  //approxCourve= cv2.approxPolyDP(curve,epsilon,closed)

  vector<Point2f> pcPoints(4);
  for(int pointI = 0;pointI < 4;pointI++){
    float picX = rectPoints[pointI].x;
    float picY = rectPoints[pointI].y;
    float rmX = picX/picScale;  
    float rmY = picY/picScale;
    float pcX = rmX - roiM/2;
    float pcY = rmY - roiM/2;
    Point2f point(pcX, pcY);
    pcPoints[pointI] = point;
  }
  PointCloud<PointXYZ> oneBbox;
  for(int pclH = 0; pclH < 2; pclH++){ 
    for(int pclP = 0; pclP < 4; pclP++){
      PointXYZ o;
      o.x = pcPoints[pclP].x;
      o.y = pcPoints[pclP].y;
      if(pclH == 0) o.z = min[2];  
      else o.z = max[2];  
      oneBbox.push_back(o);
    }
  }  

  visualization_msgs::Marker line_list; 
  line_list.header.frame_id = f_id;   
  line_list.header.stamp = ros::Time::now();
  line_list.ns =  "test_boxes";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = id;
  line_list.type = visualization_msgs::Marker::LINE_LIST; 

  //LINE_LIST markers use only the x component of scale, for the line width  仅将比例的x分量用于线宽
  line_list.scale.x = 0.01;
  // Points are green
  if(colour == 1){line_list.color.g = 1.0f;}//绿色
  else if (colour == 2){line_list.color.r = 1.0f;}//红色
  //else{line_list.color.b = 1.0f;}//蓝色
  line_list.color.a = 1.0;

    for(int pointI = 0; pointI < 4; pointI++){ 
      geometry_msgs::Point p;  
      p.x = oneBbox[pointI].x;
      p.y = oneBbox[pointI].y;
      p.z = oneBbox[pointI].z;
      line_list.points.push_back(p);  
      p.x = oneBbox[(pointI+1)%4].x;  
      p.y = oneBbox[(pointI+1)%4].y;
      p.z = oneBbox[(pointI+1)%4].z;
      line_list.points.push_back(p);

      p.x = oneBbox[pointI].x;
      p.y = oneBbox[pointI].y;
      p.z = oneBbox[pointI].z;
      line_list.points.push_back(p);
      p.x = oneBbox[pointI+4].x;
      p.y = oneBbox[pointI+4].y;
      p.z = oneBbox[pointI+4].z;
      line_list.points.push_back(p);

      p.x = oneBbox[pointI+4].x;
      p.y = oneBbox[pointI+4].y;
      p.z = oneBbox[pointI+4].z;
      line_list.points.push_back(p);
      p.x = oneBbox[(pointI+1)%4+4].x;
      p.y = oneBbox[(pointI+1)%4+4].y;
      p.z = oneBbox[(pointI+1)%4+4].z;
      line_list.points.push_back(p);
    }
  return line_list; 
}
////////////////////////////////////////////////////////////////////////

////2.地面去除 硬编码
void MovingObjectDetectionCloud::groundPlaneRemoval(float x,float y,float z)
{
    /*Hard coded ground plane removal*/

	  pcl::PassThrough<pcl::PointXYZI> pass;//直通滤波 简单过滤
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.filter(*raw_cloud);
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.filter(*raw_cloud);
    /*The pointcloud becomes more sparse as the distance of sampling from the lidar increases.
    So it has been trimmed in X,Y and Z directions
    随着距离激光雷达采样距离的增加，点云变得更加稀疏。所以它在X、Y和Z方向被修剪*/

    pcl::CropBox<pcl::PointXYZI> cropBoxFilter (true);//类CropBox过滤掉在用户给定立方体内的点云数据
    cropBoxFilter.setInputCloud(raw_cloud);
    Eigen::Vector4f min_pt(-x, -y, gp_limit, 1.0f);//立方体对角点1，(-6，-6，-0.3)
    Eigen::Vector4f max_pt(x, y, z, 1.0f);//立方体对角点2，(6，6，5)
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    //cropBoxFilter.setNegative(true);//false是只将立方体内的点保留，默认false
    cropBoxFilter.filter(*cloud); //'cloud' stores the pointcloud after removing ground plane
    gp_indices = cropBoxFilter.getRemovedIndices();
    /*ground plane is removed from 'raw_cloud' and their indices are stored in gp_indices
    gp_indices内存储地平面的索引，CropBox使用该索引将raw_cloud过滤至无地面的cloud*/
    //---------------------添加的----------------------------
    pcl::toROSMsg(*cloud, output_rgp);
    //output_rgp.header.frame_id = "gpr";
    //----------------------------------------------------------

}

//对3可选 地面去除 体素协方差 只用x y
void MovingObjectDetectionCloud::groundPlaneRemoval(float x,float y)
{
    /*Voxel covariance based ground plane removal.*/

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-x, x);
    pass.filter(*raw_cloud);
    pass.setInputCloud(raw_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-y, y);
    pass.filter(*raw_cloud);
    /*The pointcloud becomes more sparse as the distance of sampling from the lidar increases.
    So it has been trimmed in X,Y and Z directions*/

    pcl::PointCloud<pcl::PointXYZI>::Ptr dsc(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    /*pointcloud variables*/

    pcl::VoxelGrid<pcl::PointXYZI> vg; //voxelgrid filter to downsample input cloud
    vg.setInputCloud(raw_cloud);
    vg.setLeafSize(gp_leaf,gp_leaf,gp_leaf);
    vg.filter(*dsc); //'dsc' stores the downsampled pointcloud

    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr xyzi_tree(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    xyzi_tree->setInputCloud(raw_cloud); //kdtree to search for NN of the down sampled cloud points

    std::vector<std::vector<int>> index_bank;
    /*stores the index of all the points in 'raw_cloud' which satisfies the covariance condition*/

    for(int i=0;i<dsc->points.size();i++)
    {
      std::vector<int> ind;
      std::vector<float> dist;
      if(xyzi_tree->radiusSearch(dsc->points[i], gp_leaf, ind, dist) > 0 )
      //if(xyzi_tree->nearestKSearch(dsc->points[i], 20, ind, dist) > 0 )
      {
        /*this can be radius search or nearest K search. most suitable one should be considered
        according to the results after experiments*/

        if(ind.size()>3) //atleast 3 points required for covariance matrix calculation
        {
          pcl::PointCloud<pcl::PointXYZI> temp;
          for(int j=0;j<ind.size();j++)
          {
            temp.points.push_back(raw_cloud->points[ind[j]]);
          }//put all NN (inside the voxel) into a temporary pointcloud
          temp.width = temp.points.size();
          temp.height = 1;

          Eigen::Vector4f cp;
          pcl::compute3DCentroid(temp, cp); //calculate centroid
          Eigen::Matrix3f covariance_matrix;
          pcl::computeCovarianceMatrix(temp, cp, covariance_matrix); //calculate 3D covariance matrix(3x3)
          if(fabs(covariance_matrix(0,2))<0.001 && fabs(covariance_matrix(1,2))<0.001 && fabs(covariance_matrix(2,2))<0.001)
          {
            /*
            xx|xy|xz
            yx|yy|yz
            zx|zy|zz
            covariance matrix: xz,yz and zz values should be less than a threshold.
            thresholds can be modified for better results depending on the type of pointcloud.
            */
            f_cloud->points.push_back(dsc->points[i]);
            index_bank.push_back(ind);
          }
        }
      }
    }

    std::unordered_map<float,std::vector<int>> bins;//hash table类型变量
    /*a bin holds all points having Z coordinate within a specific range*/
    
    for(int i=0;i<f_cloud->points.size();i++)
    {
      float key = (float)((int)(f_cloud->points[i].z*10))/bin_gap; //bin gap for the binning step //保留一位小数
      bins[key].push_back(i);
    }
    float tracked_key = bins.begin()->first;
    int mode = bins.begin()->second.size();
    for(std::unordered_map<float,std::vector<int>>::iterator it=bins.begin();it!=bins.end();it++)
    {
      if(it->second.size()>mode)
      {
        mode = it->second.size();
        tracked_key = it->first;
      }
    }
    /*search for the bin holding highest number of points. it is supposed to be the dominating 
    plane surface*/

    boost::shared_ptr<std::vector<int>> gp_i;
    pcl::PointIndicesPtr ground_plane(new pcl::PointIndices);
    for(int i=0;i<bins[tracked_key].size();i++)
    {
      for(int j=0;j<index_bank[bins[tracked_key][i]].size();j++)
      {
        gp_i->push_back(index_bank[bins[tracked_key][i]][j]); //store the ground plane point indices in 'gp_indices'
        ground_plane->indices.push_back(index_bank[bins[tracked_key][i]][j]);
      }
    }
    gp_indices = gp_i;

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(raw_cloud);
    extract.setIndices(ground_plane);
    extract.setNegative(true);
    extract.filter(*cloud);
    /*filter the pointcloud by removing ground plane*/
}

//3.计算最新点云中的聚类
void MovingObjectDetectionCloud::computeClusters(float distance_threshold, std::string f_id)
{
	clusters.clear();
	cluster_indices.clear();
	detection_results.clear();
  centroid_collection.reset(new pcl::PointCloud<pcl::PointXYZ>);
  #ifdef VISUALIZE
	cluster_collection.reset(new pcl::PointCloud<pcl::PointXYZI>);
	#endif
  /*initialize and clear the required variables*/

    //static double start, time_taken,end;
    //start = ros::Time::now().toSec();
  	/* //TEST1 基于欧式距离提取集群的方法
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    string s_method = "EuclideanClusterExtraction:     ";
    ec.setClusterTolerance(distance_threshold);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);   */
	/*euclidian clustering*/
    
    //TEST2 基于联通组件的聚类方法
    componentClustering(cloud,cluster_indices);
    string s_method = "2.5D component_clustering:     ";   

  /* //TEST3 原始DBSCAN聚类方法
    DBSCAN_Clustering(cloud,cluster_indices);//
    string s_method = "DBSCAN_clustering:     ";    */

  /* //TEST4 KDTree加速的DBSCAN聚类方法
    string s_method = "DBSCAN_KDTree_clustering:     ";   
    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0;i<cloud->points.size();i++){
      pcl::PointXYZ o;
      o.x = cloud->points[i].x;
      o.y = cloud->points[i].y;
      o.z = cloud->points[i].z;
      if(o.x<-20||o.x>20||o.y<-20||o.y>20){continue;}
      keypoints_ptr->points.push_back(o);
    }
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(keypoints_ptr);
    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(30);//最小聚类核心数
    ec.setClusterTolerance(0.3);//距离
    ec.setMinClusterSize(40);//最小聚类点数
    ec.setMaxClusterSize(25000);//最大聚类点数
    ec.setSearchMethod(tree);//输入树
    ec.setInputCloud(keypoints_ptr);//输入点云
    ec.extract(cluster_indices);//输出存储索引 */

    /* end = ros::Time::now().toSec();
    time_taken = end - start;
    ofstream time_txt("/home/wyw/clustering_time.txt", std::ios::app);
    time_txt<<"Frame"<<+"\n";
    time_txt<<s_method<<time_taken<<+"\n";
    time_txt.close();  */
  

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
       //temporary variable临时变量
	    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
	    {
        #ifdef VISUALIZE
	    	cluster_collection->points.push_back(cloud->points[*pit]);
        #endif
	    	cloud_cluster->points.push_back(cloud->points[*pit]); //extract the cluster into 'cloud_cluster'
	    }

	    cloud_cluster->header.frame_id = f_id;
	    cloud_cluster->width = cloud_cluster->points.size ();
	    cloud_cluster->height = 1;
	    cloud_cluster->is_dense = true;

	    clusters.push_back(cloud_cluster); //add the cluster to a collection vector

	    Eigen::Vector4d temp;
	    pcl::compute3DCentroid(*cloud_cluster, temp); //compute centroid of the cluster计算质心
	    pcl::PointXYZ centroid;
	    centroid.x = temp[0]; centroid.y = temp[1]; centroid.z = temp[2];
	    centroid_collection->points.push_back(centroid); //add the centroid to a collection vector
  	}

  centroid_collection->width = centroid_collection->points.size();
  centroid_collection->height = 1;
  centroid_collection->is_dense = true;

  	for(int i=0;i<clusters.size();i++)
  	{
  		detection_results.push_back(false); 
      /*assign the moving detection results for all clusters as false initially
      最初将所有簇的移动检测结果指定为false*/
  	}

  #ifdef VISUALIZE 
  /*visualize the clustering results if VISUALIZE flag is defined
  如果定义了可视化标志，则可视化聚类结果*/
  cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
  #endif
}

//5.检查两个对应的点云的体积是否近似相等
bool MovingObjectDetectionMethods::volumeConstraint(pcl::PointCloud<pcl::PointXYZI>::Ptr fp, 
pcl::PointCloud<pcl::PointXYZI>::Ptr fc,double threshold)
{
    /*check if two corresponding pointclouds have nearly equal volume*/

    Eigen::Vector4f min;
  	Eigen::Vector4f max;
  	double volp,volc;

  	pcl::getMinMax3D(*fp, min, max);
  	volp = (max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]);
  	pcl::getMinMax3D(*fc, min, max);
  	volc = (max[0]-min[0])*(max[1]-min[1])*(max[2]-min[2]);

  	if((abs(volp-volc)/(volp+volc))<threshold) 
  	{
      /*normalized volume difference should be less than threshold*/
  		return true;
  	}
  	return false;
}

//4.查找两个连续帧之间的簇质心之间的对应关系
void MovingObjectDetectionMethods::calculateCorrespondenceCentroid(
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2,
  pcl::PointCloud<pcl::PointXYZ>::Ptr fp, pcl::PointCloud<pcl::PointXYZ>::Ptr fc, pcl::CorrespondencesPtr fmp,
  double delta)
{
  /*finds the correspondence among the cluster centroids between two consecutive frames*/

	pcl::CorrespondencesPtr ufmp(new pcl::Correspondences());

	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> corr_est;
  corr_est.setInputSource(fp);
  corr_est.setInputTarget(fc);
	corr_est.determineReciprocalCorrespondences(*ufmp);
  //类CorrespondenceEstimation是确定目标和查询点集(或特征)之间的对应关系的基类
  //输出两组点云之间对应点集合
  /*euclidian distance based reciprocal correspondence (one to one correspondence)
  基于欧几里得距离的相互对应（一对一对应*/

	for(int j=0;j<ufmp->size();j++)//逐个检查前后两帧各对应聚类的体积是否匹配
  	{
      /*filter the correspondences based on volume constraints and store in 'fmp'
      根据体积限制过滤对应关系并存储在“fmp”中*/
	    if(!volumeConstraint(c1[(*ufmp)[j].index_query],c2[(*ufmp)[j].index_match],volume_constraint))
	    {
	      continue;
	    }

	    fmp->push_back((*ufmp)[j]);
  	}
}

//6.构建源云和目标云的八叉树表示。返回相对于前帧各对应聚类点云中新增点的数量
std::vector<double> MovingObjectDetectionMethods::getClusterPointcloudChangeVector
(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, 
pcl::CorrespondencesPtr mp,float resolution = 0.3f)
{
  /*builds the octree representation of the source and destination clouds. finds the
  number of new points appearing in the destination cloud with respect to the source
  cloud. repeats this for each pair of corresponding pointcloud clusters
  构建源云和目标云的八叉树表示。查找目标云中相对于源云中出现的新点的数量。
  对每对对应的点云群集重复此操作*/

  std::vector<double> changed;
  srand((unsigned int)time(NULL));//为使用rand产生随机数作前提
  for(int j=0;j<mp->size();j++)
  {
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree_cd(resolution);//resolution 八叉树体素的边长
    octree_cd.setInputCloud(c1[(*mp)[j].index_query]);
    octree_cd.addPointsFromInputCloud();
    octree_cd.switchBuffers();//交换八叉树缓存
    octree_cd.setInputCloud(c2[(*mp)[j].index_match]);
    octree_cd.addPointsFromInputCloud();

    
    std::vector<int> newPointIdxVector;
    /*stores the indices of the new points appearing in the destination cluster
    存储目标群集中出现的新点的索引*/

    octree_cd.getPointIndicesFromNewVoxels(newPointIdxVector);//对比获得新增点的索引
    changed.push_back(newPointIdxVector.size());
  }
  return changed;
  /*return the movement scores*/
}

//对6可选 查找点从源到目标点云的对应关系。过滤距离在特定距离范围内的对应关系
std::vector<double> MovingObjectDetectionMethods::getPointDistanceEstimateVector
(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, 
pcl::CorrespondencesPtr mp)
{
  /*finds the correspondence of points from source to destination pointcloud.filters the
  correspondences having distance within a specific distance range. repeats this for each
  pair of corresponding pointcloud clusters
  查找点从源到目标点云的对应关系。过滤距离在特定距离范围内的对应关系。
  对每对对应的点云群集重复此操作*/

	std::vector<double> estimates;
 	pcl::registration::CorrespondenceEstimation<pcl::PointXYZI, pcl::PointXYZI> corr_est;
  pcl::CorrespondencesPtr corrs;    
	for(int j=0;j<mp->size();j++)
	{
		corrs.reset(new pcl::Correspondences());
    corr_est.setInputSource(c1[(*mp)[j].index_query]);
  	corr_est.setInputTarget(c2[(*mp)[j].index_match]);
		corr_est.determineCorrespondences(*corrs);
    /*euclidian distance based correspondence (one to many correspondence)*/

		double count = 0;
		for(int i=0;i<corrs->size();i++)
		{
			if((*corrs)[i].distance>pde_lb && (*corrs)[i].distance<pde_ub)
			{
				count++;
			}
		}
		estimates.push_back(count/((c1[(*mp)[j].index_query]->points.size()+c2[(*mp)[j].index_match]->points.size())/2));
		/*normalize 'count' with respect to the size of corresponding clusters*/
	}
	return estimates;
  /*return the movement scores*/
}

//0.初始化列表
MovingObjectRemoval::MovingObjectRemoval(ros::NodeHandle nh_,std::string config_path,int n_bad,int n_good):nh(nh_),moving_confidence(n_bad),static_confidence(n_good)
{
    setVariables(config_path);

    /*ROS setup*/
    #ifdef VISUALIZE
    pub = nh.advertise<sensor_msgs::PointCloud2> (output_topic, 10);
    debug_pub = nh.advertise<sensor_msgs::PointCloud2> (debug_topic, 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>(marker_topic, 10);
    marker1_pub = nh.advertise<visualization_msgs::Marker>("static01",10);
    marker2_pub = nh.advertise<visualization_msgs::Marker>("All_cluster",10);
    marker3_pub = nh.advertise<visualization_msgs::Marker>("distance",10);
    marker4_pub = nh.advertise<visualization_msgs::Marker>("velocity",1);
    gpr = nh.advertise<sensor_msgs::PointCloud2>("aaa",10);

    #endif

    #ifdef INTERNAL_SYNC
    pc_sub.subscribe(nh, input_pointcloud_topic, 1);
    odom_sub.subscribe(nh, input_odometry_topic, 1);
    sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10),pc_sub,odom_sub));
    sync->registerCallback(&MovingObjectRemoval::movingCloudObjectSubscriber, this);
    /*internal message synchronization using ROS Approximate Time policy
    使用ROS近似时间策略的内部消息同步*/
    #endif

    ca.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //previous pointcloud frame
    cb.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //current pointcloud frame (latest)
    mth.reset(new MovingObjectDetectionMethods(volume_constraint,pde_lb,pde_ub));
    /*instantiate the shared pointers实例化共享指针*/
}

//0.2 内部同步订阅服务器 INTERNAL_SYNC标志控制
void MovingObjectRemoval::movingCloudObjectSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
  /*subscriber for internal sync. works if INTERNAL_SYNC flag is defined
  内部同步订阅服务器。如果定义了INTERNAL\u SYNC标志，则工作*/

  clock_t begin_time = clock();
  std::cout<<"-----------------------------------------------------\n";
  pcl::PCLPointCloud2 cloud;
  pcl_conversions::toPCL(*input, cloud);

  pushRawCloudAndPose(cloud,odm->pose.pose);
  if(filterCloud(cloud,output_fid))
  {
    #ifdef VISUALIZE
    pub.publish(output);
    #endif
  }

  std::cout<<1000.0*(clock()-begin_time)/CLOCKS_PER_SEC<<std::endl;
  /*print CPU time taken by the algorithm per iteration*/

  std::cout<<"-----------------------------------------------------\n";
}

//8.将对应映射缓冲区索引和结果缓冲区索引作为初始参数，并递归到缓冲区中所有可用的对应映射结束。
int MovingObjectRemoval::recurseFindClusterChain(int col,int track)
{
  /*takes the correspondence map buffer index and result buffer index as initial parameter
  and recurses till the end of all the correspondence maps available in the buffer. cluster chain
  information is obtained from the correspondence map buffer 'corrs_vec'. consistency in the 
  cluster chain is known using the result buffer 'res_vec'. returns -1 if consistency fails
  or else returns the index of the moving cluster in the cluster collection 'clusters' in
  the latest pointcloud frame 'cb'
  将对应映射缓冲区索引和结果缓冲区索引作为初始参数，并递归到缓冲区中所有可用的对应映射结束。
  簇链信息从对应映射缓冲区“corrs_vec”获取。使用结果缓冲区“res_vec”可以知道集群链中的一致性。
  如果一致性失败，则返回-1，否则返回最新点云框架“cb”中群集集合“clusters”中移动群集的索引*/

  if(col == corrs_vec.size())
  {
    /*break condition for the recursion. return the index of the moving cluster
    递归的中断条件。返回移动群集的索引*/
    return track;
  }

  for(int j=0;j<corrs_vec[col]->size();j++)
  {
    /*search all the unit correspondeces within the correspondence map in 'col' index of the buffer
    在缓冲区的“col”索引中的对应关系图中搜索所有单位对应关系*/
    
    if((*corrs_vec[col])[j].index_query == track)
    {
      /*correspondence map should have the key 'track'通信地图应具有关键“track”*/

      if(res_vec[col+1][(*corrs_vec[col])[j].index_match] == true)
      {
        /*the mapping index must have a true positive value in the result buffer
        映射索引在结果缓冲区中必须具有true值*/

        return recurseFindClusterChain(col+1,(*corrs_vec[col])[j].index_match);
        /*if both key and mapped index have true positive value then move for the next correspondence
        map in the buffer如果键和映射索引都具有真正值，则移动到缓冲区中的下一个对应映射*/
      }
      else
      {
        return -1;
      }
    }
  }
  return -1;
}

//9.用于检查移动质心并将其推送到已确认的移动簇向量的函数
void MovingObjectRemoval::pushCentroid(pcl::PointXYZ pt)
{
  /*function to check and push the moving centroid to the confirmed moving cluster vector*/

	for(int i=0;i<mo_vec.size();i++)
	{
    /*check if the moving centroid has already been added to the 'mo_vec' previously
    检查之前是否已将移动质心添加到“mo_vec”*/
		double dist = sqrt(pow(pt.x-mo_vec[i].centroid.x,2)+pow(pt.y-mo_vec[i].centroid.y,2)+pow(pt.z-mo_vec[i].centroid.z,2));
		if(dist<catch_up_distance)
		{
      /*if found a centroid close to the new moving centroid then return, as no additional 
      action is required如果发现靠近新移动质心的质心，则返回，因为无需执行其他操作*/
			return;
		}
	}

	MovingObjectCentroid moc(pt,static_confidence);//static_confidence=3
  /*assign static confidence to 'moc' that determines it's persistance in 'mo_vec'
  为“moc”分配静态置信度，以确定其在“mov_vec”中的持久性*/

	mo_vec.push_back(moc);
  /*if not present then add the new cluster centroid to the 'mo_vec'
  如果不存在，则将新簇质心添加到“movec”*/
}

//7.在检测步骤之后获取新的对应关系图和结果向量，并更新缓冲区
void MovingObjectRemoval::checkMovingClusterChain(pcl::CorrespondencesPtr mp,std::vector<bool> &res_ca,
std::vector<bool> &res_cb)
{
  /*gets new correspondence map and result vector after the detection step and updates the
  buffers. it checks for new moving clusters and adds them to the 'mo_vec'
  在检测步骤之后获取新的对应关系图和结果向量，并更新缓冲区。
  它检查新的移动群集并将其添加到“mo_vec*/

  corrs_vec.push_back(mp);
  if(res_vec.size()==0)
  {
  //   res_vec.pop_back(); //deletes the top most result in the buffer删除缓冲区中最顶端的结果
    res_vec.push_back(res_ca);
  }
  // res_vec.push_back(res_ca); //updates the buffer with the latest result使用最新结果更新缓冲区
  res_vec.push_back(res_cb);
  // std::cout<<mo_vec.size()<<" "<<corrs_vec.size()<<" "<<res_vec.size()<<std::endl;
  if(res_vec.size() >= moving_confidence)//moving_confidence=4
  {
    for(int i=0;i<res_vec[0].size();i++)
    {
      if(res_vec[0][i] == true)
      {
        /*look to the historical data in the result buffer and check the clusters with true positive
        value in the first result vector within the buffer
        查看结果缓冲区中的历史数据，并检查缓冲区中第一个结果向量中具有真正值的聚类*/

      int found_moving_index = recurseFindClusterChain(0,i);
        /*run the recursive test to find a potential cluster chain form the buffers
        运行递归测试以从缓冲区中找到潜在的集群链*/

        if(found_moving_index != -1)
        {
          /*if found push the confirmed moving centroid into 'mo_vec'
          如果找到，将确认的移动质心移入“mo_vec”*/
          pushCentroid(cb->centroid_collection->points[found_moving_index]);
        }
      }
    }
    corrs_vec.pop_front(); //delete old values from the buffer将前第5帧从队列中删除
    res_vec.pop_front();
  }
}

int counta = 0;
//1.接收同步的传入数据并运行检测方法
void MovingObjectRemoval::pushRawCloudAndPose(pcl::PCLPointCloud2 &in_cloud,geometry_msgs::Pose pose)
{
  /*recieves the synchronized incoming data and runs detection methods*/

  ca = cb; //update previous frame with the current frame 将前一帧计算结果cb传递至ca保存
  cb.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); 
  //释放原来的空间 默认delete,cb清零

  pcl::fromPCLPointCloud2(in_cloud, *(cb->raw_cloud)); //load latest pointcloud
 /*  ofstream time_txt("/home/wyw/pose_test03.txt", std::ios::app);
  time_txt<<"Frame"<<counta<<+"\n";
  time_txt<<pose<<+"\n";
  time_txt.close();
  counta++; */
 
    
  /* ofstream time_txt("/home/wyw/pose_test03.txt", std::ios::app);
  time_txt<<"Frame"<<counta<<+"\n";
  time_txt<<pose<<+"\n";
  time_txt.close(); */
  tf::poseMsgToTF(pose,cb->ps); //load latest pose
  

  cb->groundPlaneRemoval(trim_x,trim_y,trim_z); //ground plane removal (hard coded)地面去除 硬编码
  //cb->groundPlaneRemoval(trim_x,trim_y); //groud plane removal (voxel covariance)地面去除 体素协方差
  gpr.publish(cb->output_rgp);

  cb->computeClusters(ec_distance_threshold,"single_cluster"); 
  /*compute clusters within the lastet pointcloud计算最新点云中的聚类*/

  showDistance(cb->centroid_collection);
  //显示聚类中心的距离
  

 //输出所有聚类
  float rd02=0,gd02=1.0,bd02=0;
  for (int i = 0; i < cb->clusters.size(); i++)
  {
		//marker2_pub.publish(mark_cluster(cb->clusters[i],i,debug_fid,"All_box",rd02,gd02,bd02));
    marker2_pub.publish(mark_cluster2(cb->clusters[i],i,debug_fid,1));
  }

  cb->init = true; //confirm the frame for detection确认检测框架

  if(ca->init  == true && cb->init == true)
  {
    tf::Transform t = (cb->ps).inverseTimes(ca->ps); 
    /*calculate transformation matrix between previous and current pose. 't' transforms a point
    from the previous pose to the current pose计算上一个姿势和当前姿势之间的变换矩阵。”t’
    将点从上一个姿势转换为当前姿势*/


    pcl::PointCloud<pcl::PointXYZ> temp = *ca->centroid_collection;
	  pcl_ros::transformPointCloud(temp,*ca->centroid_collection,t);
    /*transform the previous centroid collection with respect to 't'
    相对于“t”变换以前的质心集合*/

	for(int i=0;i<ca->clusters.size();i++)
	{
    /*transform the clusters in the collection vector of the previous frame with respect to 't'
    根据“t”变换前一帧集合向量中的簇*/

		pcl::PointCloud<pcl::PointXYZI> temp;
		temp = *ca->clusters[i];
	  pcl_ros::transformPointCloud(temp,*ca->clusters[i],t);
	}
  //至此，ca中所存储的上一帧点云的聚类点云和质心坐标使用pose信息消除了车辆自身移动的影响
  #ifdef VISUALIZE //visualize the cluster collection if VISUALIZE flag is defined
	pcl::toPCLPointCloud2(*cb->cluster_collection,in_cloud);
	pcl_conversions::fromPCL(in_cloud, output);
	output.header.frame_id = debug_fid;
	debug_pub.publish(output);
  #endif
  		
	pcl::CorrespondencesPtr mp(new pcl::Correspondences()); 
  /*correspondence map between the cluster centroids of previous and current frame
   前一帧与当前帧簇质心的对应关系图*/
	  	
	//cluster correspondence methods (Global)聚类对应方法（全局）
	mth->calculateCorrespondenceCentroid(ca->clusters,cb->clusters,ca->centroid_collection,cb->centroid_collection,mp,0.1);
	/*calculate euclidian correspondence and apply the voulme constraint
  计算欧几里得对应关系并应用voulme约束
  对于体积差超出阈值的聚类点云的对应关系在mp中删除*/

	//moving object detection methods (Local)运动目标检测方法（局部）
  std::vector<double> param_vec;
  if(method_choice==1)
	{
    param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);

	}
  else if(method_choice==2)
  {
    param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);
    //向量param_vec存储相对于前帧各对应聚类点云中新增点的数量
  }
  /*determine the movement scores for the corresponding clusters
  确定相应集群的移动分数*/
  // int id=1;
  // static int ct = 0;
	for(int j=0;j<mp->size();j++)
	{
		//std::cout<<"{"<<(*mp)[j].index_query<<"->"<<(*mp)[j].index_match<<"} Fit Score: "<<(*mp)[j].distance<<" Moving_Score: "<<param_vec[j]<<std::endl;
		double threshold;
    if(method_choice==1)
    {
      threshold = pde_distance_threshold;
		}
    else if(method_choice==2)
    {
      threshold = (ca->clusters[(*mp)[j].index_query]->points.size()+
                              cb->clusters[(*mp)[j].index_match]->points.size())/opc_normalization_factor;
    }

		if(param_vec[j]>threshold)
		{
			//ca->detection_results[(*mp)[j].index_query] = true;
			cb->detection_results[(*mp)[j].index_match] = true;
			//marker_pub.publish(mark_cluster(cb->clusters[(*mp)[j].index_match],id++,debug_fid,"bounding_box",0.8,0.1,0.4));
			//ct++;
		}
		else
		{
			//ca->detection_results[(*mp)[j].index_query] = false;
			cb->detection_results[(*mp)[j].index_match] = false;
		}
    /*assign the boolean results acording to thresholds. true for moving and false for static cluster
    根据阈值分配布尔结果。对于移动集群为true，对于静态集群为false*/
	}
	// std::cout<<ct<<std::endl;
  //显示速度
  showV(ca->centroid_collection,cb->centroid_collection,mp,cb->detection_results);

	checkMovingClusterChain(mp,ca->detection_results,cb->detection_results);
  /*submit the results to update the buffers
  提交结果以更新缓冲区*/
    /* static double start, time_taken,end;
    start = ros::Time::now().toSec();
    end = ros::Time::now().toSec();
    time_taken = end - start;
    ofstream time_txt("/home/wyw/checkMovingClusterChain.txt", std::ios::app);
    time_txt<<"Frame"<<+"\n";
    time_txt<<time_taken<<+"\n";
    time_txt.close();   */
  }
}

//10 从mo_ec中移除静态聚类对象，在最新点云中移除移动动态物体，output输出过滤后的点云 
bool MovingObjectRemoval::filterCloud(pcl::PCLPointCloud2 &out_cloud,std::string f_id)
{
  /*removes the moving objects from the latest pointcloud and puts the filtered cloud in 'output'.
  removes the static cluster centroids from 'mo_vec'
  从最新的点云中删除移动对象，并将过滤后的云置于“输出”中。从“mo_vec”中删除静态簇质心
  */

	xyz_tree.setInputCloud(cb->centroid_collection); 
  /*use kdtree for searching the moving cluster centroid within 'centroid_collection' of the
  latest frame使用kdtree在最新帧的“centroid_collection”中搜索移动的群集质心*/

	float rd=0.8,gd=0.1,bd=0.4;int id = 1; //colour variables for visualizing red bounding box
  float rd01=0,gd01=1.0,bd01=0;
  pcl::PointIndicesPtr moving_points(new pcl::PointIndices);
  /*stores the indices of the points belonging to the moving clusters within 'cloud'
  存储属于“cloud”中移动簇的点的索引*/

  // static int ct=0;
  // ct+=mo_vec.size();
  // std::cout<<ct<<std::endl;
  std::vector<int> movingclusterid;
	for(int i=0;i<mo_vec.size();i++)
	{
    /*iterate through all the moving cluster centroids遍历所有移动的簇质心*/

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		if(xyz_tree.nearestKSearch(mo_vec[i].centroid, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
      /*search for the actual centroid in the centroid collection of the latest frame
      在最新帧的质心集合中搜索实际质心*/

      #ifdef VISUALIZE //visualize bounding box to show the moving cluster if VISUALIZE flag is defined
			marker_pub.publish(mark_cluster2(cb->clusters[pointIdxNKNSearch[0]],id++,debug_fid,2));
			#endif
      //---------------------------------------------------------------------
      movingclusterid.push_back(pointIdxNKNSearch[0]);
      //---------------------------------------------------------------------
      
      for(int j=0;j<cb->cluster_indices[pointIdxNKNSearch[0]].indices.size();j++)
      {
        /*add the indices of the moving clusters in 'cloud' to 'moving_points'
        将“cloud”中移动簇的索引添加到“moving_points”*/
        moving_points->indices.push_back(cb->cluster_indices[pointIdxNKNSearch[0]].indices[j]);
      }

			if(cb->detection_results[pointIdxNKNSearch[0]] == false || pointNKNSquaredDistance[0]>leave_off_distance)
			{
        /*decrease the moving confidence if the cluster is found to be static in the latest results or
        if the cluster dosen't appear in the current frame
      如果在最新结果中发现群集是静态的，或者如果群集没有出现在当前帧中，则降低移动置信度*/

				if(mo_vec[i].decreaseConfidence())
				{
          /*remove the moving centroid from 'mo_vec' if the confidence reduces to 0
          如果置信度降至0，则从“mo_vec”中删除移动质心*/
					mo_vec.erase(mo_vec.begin()+i);
					i--;
				}
			}
			else
			{
				mo_vec[i].centroid = cb->centroid_collection->points[pointIdxNKNSearch[0]];
        /*update the moving centroid with the latest centroid of the moving cluster
        使用移动簇的最新质心更新移动质心*/

				mo_vec[i].increaseConfidence(); //increase the moving confidence提高移动置信度
			}
			id++;
		}
  }

  int id1 = 10;
  for (int i = 0; i < cb->clusters.size(); i++)
  { 
    if (compare(i,movingclusterid))
    {
     #ifdef VISUALIZE //visualize bounding box to show the moving cluster if VISUALIZE flag is defined
			marker1_pub.publish(mark_cluster2(cb->clusters[i],id1++,debug_fid,1));
			#endif
    }
  }  


  pcl::PointCloud<pcl::PointXYZI>::Ptr f_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cb->cloud);
  extract.setIndices(moving_points);
  extract.setNegative(true);
  extract.filter(*f_cloud);
  /*extract the moving clusters from 'cloud' and assign the filtered cloud to 'f_cloud'
  从“cloud”中提取移动的群集，并将过滤后的云指定给“f_cloud”*/

  for(int i=0;i<cb->gp_indices->size();i++)
  {
    f_cloud->points.push_back(cb->raw_cloud->points[cb->gp_indices->at(i)]);
  }
  f_cloud->width = f_cloud->points.size();
  f_cloud->height = 1;
  f_cloud->is_dense = true;
  /*merge the ground plane to the filtered cloud
  将地平面合并到过滤云*/

  pcl::toPCLPointCloud2(*f_cloud,out_cloud);
  pcl_conversions::fromPCL(out_cloud, output);
  output.header.frame_id = f_id;
  /*assign the final filtered cloud to the 'output'将最终过滤的云指定给“output”*/

  return true; //confirm that a new filtered cloud is available确认新的过滤云可用
}


int MovingObjectRemoval::compare(int a,std::vector<int>b)
{
    for (int i = 0; i < b.size(); i++)
    {
      if (a==b[i])
      {
        return 0;
      }
    }
    return 1;
}

//显示聚类中心距离
void MovingObjectRemoval::showDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr &c2){
  std::vector<float> distance_;
  for(int i=0;i<c2->size();i++){
    float dist =  sqrt(pow(c2->points[i].x,2)+pow(c2->points[i].y,2)+pow(c2->points[i].z,2));
    distance_.push_back(dist);
  }
  for(int i=0;i<c2->size();i++){
    visualization_msgs::Marker Marker_i;
    Marker_i.header.frame_id = debug_fid;
    Marker_i.header.stamp = ros::Time::now(); 
    Marker_i.action = visualization_msgs::Marker::ADD; 
    Marker_i.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    Marker_i.ns="distance_";
    Marker_i.pose.orientation.w=1.0;
    Marker_i.id=i;
    Marker_i.scale.x=0.3;
    Marker_i.scale.y=0.3;
    Marker_i.scale.z=0.3;//文字的大小
    Marker_i.color.b=25;
    Marker_i.color.g=0;
    Marker_i.color.r=25;//文字的颜色
    Marker_i.color.a=1;
    geometry_msgs::Pose pose;
    pose.position.x=c2->points[i].x;
    pose.position.y=c2->points[i].y;
    pose.position.z=c2->points[i].z;
    float string;
    string = distance_[i];
    std::ostringstream str;
    str<<string;
    Marker_i.text=str.str();
    Marker_i.pose=pose;
    marker3_pub.publish(Marker_i);
  }
}

//显示速度
void MovingObjectRemoval::showV(pcl::PointCloud<pcl::PointXYZ>::Ptr &c1,pcl::PointCloud<pcl::PointXYZ>::Ptr &c2,
pcl::CorrespondencesPtr mp,std::vector<bool> &res_cb){
  std::vector<float> velocity;
  for(int j=0;j<mp->size();j++){
    float V =  sqrt(pow(c2->points[(*mp)[j].index_match].x-c1->points[(*mp)[j].index_query].x,2)
    +pow(c2->points[(*mp)[j].index_match].y-c1->points[(*mp)[j].index_query].y,2)
    +pow(c2->points[(*mp)[j].index_match].z-c1->points[(*mp)[j].index_query].z,2))/0.1;
    velocity.push_back(V);
  }
  for(int i=0;i<mp->size();i++){
    if(res_cb[(*mp)[i].index_match]&&velocity[i]>0.5){
    visualization_msgs::Marker Marker_i;
    Marker_i.header.frame_id = debug_fid;
    Marker_i.header.stamp = ros::Time::now(); 
    Marker_i.action = visualization_msgs::Marker::ADD; 
    Marker_i.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
    Marker_i.ns="veloctiy";
    Marker_i.pose.orientation.w=1.0;
    Marker_i.id=i;
    Marker_i.scale.x=0.3;
    Marker_i.scale.y=0.3;
    Marker_i.scale.z=0.3;//文字的大小
    Marker_i.color.b=25;
    Marker_i.color.g=0;
    Marker_i.color.r=25;//文字的颜色
    Marker_i.color.a=1;
    geometry_msgs::Pose pose;
    pose.position.x=c2->points[(*mp)[i].index_match].x;
    pose.position.y=c2->points[(*mp)[i].index_match].y;
    pose.position.z=c2->points[(*mp)[i].index_match].z+1.5; 
    float string;
    string =velocity[i];
    std::ostringstream str;
    str<<string;
    Marker_i.text=str.str();
    Marker_i.pose=pose;
    Marker_i.lifetime = ros::Duration(0.3);
    marker4_pub.publish(Marker_i); 

    visualization_msgs::Marker Marker_j;
    Marker_j.header.frame_id = debug_fid;
    Marker_j.header.stamp = ros::Time::now(); 
    Marker_j.action = visualization_msgs::Marker::ADD; 
    Marker_j.type=visualization_msgs::Marker::ARROW;
    Marker_j.ns="veloctiy_arrow";
    Marker_j.pose.orientation.w=1.0;
    Marker_j.id=i;
    Marker_j.scale.x=0.3;
    Marker_j.scale.y=0.3;
    Marker_j.scale.z=0.3;//大小
    Marker_j.color.b=0;
    Marker_j.color.g=0;
    Marker_j.color.r=25;//颜色
    Marker_j.color.a=1;
  
    geometry_msgs::Point p1;
    p1.x=c1->points[(*mp)[i].index_query].x;
    p1.y=c1->points[(*mp)[i].index_query].y;
    p1.z=c1->points[(*mp)[i].index_query].z; 
    Marker_j.points.push_back(p1);
    geometry_msgs::Point p;
    p.x=c2->points[(*mp)[i].index_match].x+2*(c2->points[(*mp)[i].index_match].x-c1->points[(*mp)[i].index_query].x);
    p.y=c2->points[(*mp)[i].index_match].y+2*(c2->points[(*mp)[i].index_match].y-c1->points[(*mp)[i].index_query].y);
    p.z=c2->points[(*mp)[i].index_match].z; 
    Marker_j.points.push_back(p);

    Marker_j.scale.x = 0.1;
    Marker_j.scale.y = 0.2;
    Marker_j.scale.z = 2;

    Marker_j.lifetime = ros::Duration(0.4);
    marker4_pub.publish(Marker_j); 
    }
  }
}

//测试：显示多边形
/* void MovingObjectRemoval::showPolygon(){
  geometry_msg::PolygonStamped myPolygon;
  geometry_msgs::Point32 point;
  myPolygon.header.frame_id = debug_fid;
  
} */

//0.1设置变量
void MovingObjectRemoval::setVariables(std::string config_file_path)
{
  std::fstream config;
  config.open(config_file_path);

  if(!config.is_open())
  {
    std::cout<<"Couldnt open the file\n";
    exit(0);
  } //open config file

  std::string line,parm1,parm2; //required string variables
  while(std::getline(config,line)) //extract lines one by one
  {
    if(line[0]=='#' || line.length()<3)
    {
      continue;
    }
    parm1 = "";parm2="";
    bool flag = true;
    for(int ind=0;ind<line.length();ind++)
    {
      if(line[ind]==':')
      {
        flag = false;
        continue;
      }
      if(flag)
      {
        parm1.push_back(line[ind]);
      }
      else
      {
        parm2.push_back(line[ind]);
      }
    } //extract lines "name_of_variable:value"

      std::cout<<parm1<<":";
      if(parm1 == "gp_limit")
      {
        gp_limit = std::stof(parm2);
        std::cout<<gp_limit;
      }
      else if(parm1 == "gp_leaf")
      {
        gp_leaf = std::stof(parm2);
        std::cout<<gp_leaf;
      }
      else if(parm1 == "bin_gap")
      {
        bin_gap = std::stof(parm2);
        std::cout<<bin_gap;
      }
      else if(parm1 == "min_cluster_size")
      {
        min_cluster_size = std::stol(parm2);
        std::cout<<min_cluster_size;
      }
      else if(parm1 == "max_cluster_size")
      {
        max_cluster_size = std::stol(parm2);
        std::cout<<max_cluster_size;
      }
      else if(parm1 == "volume_constraint")
      {
        volume_constraint = std::stof(parm2);
        std::cout<<volume_constraint;
      }
      else if(parm1 == "pde_lb")
      {
        pde_lb = std::stof(parm2);
        std::cout<<pde_lb;
      }
      else if(parm1 == "pde_ub")
      {
        pde_ub = std::stof(parm2);
        std::cout<<pde_ub;
      }
      else if(parm1 == "output_topic")
      {
        output_topic = parm2;
        std::cout<<output_topic;
      }
      else if(parm1 == "debug_topic")
      {
        debug_topic = parm2;
        std::cout<<debug_topic;
      }
      else if(parm1 == "marker_topic")
      {
        marker_topic = parm2;
        std::cout<<marker_topic;
      }
      else if(parm1 == "input_pointcloud_topic")
      {
        input_pointcloud_topic = parm2;
        std::cout<<input_pointcloud_topic;
      }
      else if(parm1 == "input_odometry_topic")
      {
        input_odometry_topic = parm2;
        std::cout<<input_odometry_topic;
      }
      else if(parm1 == "output_fid")
      {
        output_fid = parm2;
        std::cout<<output_fid;
      }
      else if(parm1 == "debug_fid")
      {
        debug_fid = parm2;
        std::cout<<debug_fid;
      }
      else if(parm1 == "leave_off_distance")
      {
        leave_off_distance = std::stof(parm2);
        std::cout<<leave_off_distance;
      }
      else if(parm1 == "catch_up_distance")
      {
        catch_up_distance = std::stof(parm2);
        std::cout<<catch_up_distance;
      }
      else if(parm1 == "trim_x")
      {
        trim_x = std::stof(parm2);
        std::cout<<trim_x;
      }
      else if(parm1 == "trim_y")
      {
        trim_y = std::stof(parm2);
        std::cout<<trim_y;
      }
      else if(parm1 == "trim_z")
      {
        trim_z = std::stof(parm2);
        std::cout<<trim_z;
      }
      else if(parm1 == "ec_distance_threshold")
      {
        ec_distance_threshold = std::stof(parm2);
        std::cout<<ec_distance_threshold;
      }
      else if(parm1 == "opc_normalization_factor")
      {
        opc_normalization_factor = std::stof(parm2);
        std::cout<<opc_normalization_factor;
      }
      else if(parm1 == "pde_distance_threshold")
      {
        pde_distance_threshold = std::stof(parm2);
        std::cout<<pde_distance_threshold;
      }
      else if(parm1 == "method_choice")
      {
        method_choice = std::stoi(parm2);
        std::cout<<method_choice;
      }
      else
      {
        std::cout<<"Invalid parameter found in config file\n";
        exit(0);
      }
      std::cout<<std::endl;
      /*assign values to the variables based on name*/
  }
}