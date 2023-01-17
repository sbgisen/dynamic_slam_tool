#include "MOR/MovingObjectRemoval.h"

//'xyz' -> refers to the variable with name xyz

////////////////////////////////////////////////////////////////////Helping Methods

// function to generate a bounding box visualization
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

// function to generate a bounding box visualization
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

  //LINE_LIST markers use only the x component of scale, for the line width
  line_list.scale.x = 0.01;
  // Points are green
  if(colour == 1){line_list.color.g = 1.0f;}// green
  else if (colour == 2){line_list.color.r = 1.0f;}// red
  //else{line_list.color.b = 1.0f;}// blue
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

//// 2. Ground removal hardcoded
void MovingObjectDetectionCloud::groundPlaneRemoval(float x,float y,float z)
{
    /*Hard coded ground plane removal*/

	  pcl::PassThrough<pcl::PointXYZI> pass;// Straight-through filter Simple filter
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

    pcl::CropBox<pcl::PointXYZI> cropBoxFilter (true);// class CropBox to filter out the point cloud data in the cube given by the user
    cropBoxFilter.setInputCloud(raw_cloud);
    Eigen::Vector4f min_pt(-x, -y, gp_limit, 1.0f);// Cube diagonal point 1, (-6, -6, -0.3)
    Eigen::Vector4f max_pt(x, y, z, 1.0f);// cube diagonal point 2, (6, 6, 5)
    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);
    //cropBoxFilter.setNegative(true);// false is to only keep the points in the cube, the default is false
    cropBoxFilter.filter(*cloud); //'cloud' stores the pointcloud after removing ground plane
    gp_indices = cropBoxFilter.getRemovedIndices();
    /*ground plane is removed from 'raw_cloud' and their indices are stored in gp_indices*/
    // ---------------------Additional----------------------- -----
    pcl::toROSMsg(*cloud, output_rgp);
    //output_rgp.header.frame_id = "gpr";
    //----------------------------------------------------------

}

// Optional for 3 ground removal voxel covariance only use x y
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

    std::unordered_map<float,std::vector<int>> bins;//hash table
    /*a bin holds all points having Z coordinate within a specific range*/
    
    for(int i=0;i<f_cloud->points.size();i++)
    {
      float key = (float)((int)(f_cloud->points[i].z*10))/bin_gap; //bin gap for the binning step
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

// 3. Calculate the clustering in the latest point cloud
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
  	/* // TEST1 Method for extracting clusters based on Euclidean distance
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    string s_method = "EuclideanClusterExtraction:     ";
    ec.setClusterTolerance(distance_threshold);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setInputCloud(cloud);
  	ec.extract(cluster_indices);   */
	/*euclidian clustering*/
    
    // TEST2 Clustering method based on connected components
    componentClustering(cloud,cluster_indices);
    string s_method = "2.5D component_clustering:     ";   

  /* // TEST3 original DBSCAN clustering method
    DBSCAN_Clustering(cloud,cluster_indices);//
    string s_method = "DBSCAN_clustering:     ";    */

  /* // TEST4 KDTree accelerated DBSCAN clustering method
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
    ec.setCorePointMinPts(30);// Minimum number of clustering cores
    ec.setClusterTolerance(0.3);// distance
    ec.setMinClusterSize(40);// Minimum number of clustering points
    ec.setMaxClusterSize(25000);// Maximum number of clustering points
    ec.setSearchMethod(tree);// input tree
    ec.setInputCloud(keypoints_ptr);// input point cloud
    ec.extract(cluster_indices);// output storage index */

    /* end = ros::Time::now().toSec();
    time_taken = end - start;
    ofstream time_txt("/home/wyw/clustering_time.txt", std::ios::app);
    time_txt<<"Frame"<<+"\n";
    time_txt<<s_method<<time_taken<<+"\n";
    time_txt.close();  */
  

  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
       //temporary variable
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
	    pcl::compute3DCentroid(*cloud_cluster, temp); //compute centroid of the cluster
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
      /*assign the moving detection results for all clusters as false initially*/
  	}

  #ifdef VISUALIZE 
  /*visualize the clustering results if VISUALIZE flag is defined*/
  cluster_collection->width = cluster_collection->points.size();
	cluster_collection->height = 1;
	cluster_collection->is_dense = true;
  #endif
}

// 5. Check whether the volumes of the two corresponding point clouds are approximately equal
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

// 4. Find the correspondence between the cluster centroids between two consecutive frames
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
  // Class CorrespondenceEstimation is the base class for determining the correspondence between target and query point sets (or features)
  /*euclidian distance based reciprocal correspondence (one to one correspondence)*/

	for(int j=0;j<ufmp->size();j++)// Check whether the volumes of the corresponding clusters of the two frames before and after match one by one
  	{
      /*filter the correspondences based on volume constraints and store in 'fmp'*/
	    if(!volumeConstraint(c1[(*ufmp)[j].index_query],c2[(*ufmp)[j].index_match],volume_constraint))
	    {
	      continue;
	    }

	    fmp->push_back((*ufmp)[j]);
  	}
}

// 6. Construct the octree representation of the source and target clouds. Returns the number of new points in each corresponding cluster point cloud relative to the previous frame
std::vector<double> MovingObjectDetectionMethods::getClusterPointcloudChangeVector
(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, 
pcl::CorrespondencesPtr mp,float resolution = 0.3f)
{
  /*builds the octree representation of the source and destination clouds. finds the
  number of new points appearing in the destination cloud with respect to the source
  cloud. repeats this for each pair of corresponding pointcloud clusters*/

  std::vector<double> changed;
  srand((unsigned int)time(NULL));// As a prerequisite for using rand to generate random numbers
  for(int j=0;j<mp->size();j++)
  {
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree_cd(resolution);// resolution The side length of the octree voxel
    octree_cd.setInputCloud(c1[(*mp)[j].index_query]);
    octree_cd.addPointsFromInputCloud();
    octree_cd.switchBuffers();// swap octree cache
    octree_cd.setInputCloud(c2[(*mp)[j].index_match]);
    octree_cd.addPointsFromInputCloud();

    
    std::vector<int> newPointIdxVector;
    /*stores the indices of the new points appearing in the destination cluster*/

    octree_cd.getPointIndicesFromNewVoxels(newPointIdxVector);// Compare to get the index of the new point
    changed.push_back(newPointIdxVector.size());
  }
  return changed;
  /*return the movement scores*/
}

// Optional for 6 Find the correspondence of points from the source to the target point cloud. Correspondence between filter distances within a specific distance range
std::vector<double> MovingObjectDetectionMethods::getPointDistanceEstimateVector
(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c1,std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> &c2, 
pcl::CorrespondencesPtr mp)
{
  /*finds the correspondence of points from source to destination pointcloud.filters the
  correspondences having distance within a specific distance range. repeats this for each
  pair of corresponding pointcloud clusters*/

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

// 0.initialization list
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
    /*internal message synchronization using ROS Approximate Time policy*/
    #endif

    ca.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //previous pointcloud frame
    cb.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); //current pointcloud frame (latest)
    mth.reset(new MovingObjectDetectionMethods(volume_constraint,pde_lb,pde_ub));
    /*instantiate the shared pointers*/
}

// 0.2 Internal synchronization subscriber INTERNAL_SYNC flag control
void MovingObjectRemoval::movingCloudObjectSubscriber(const sensor_msgs::PointCloud2ConstPtr& input, const nav_msgs::OdometryConstPtr& odm)
{
  /*subscriber for internal sync. works if INTERNAL_SYNC flag is defined*/

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

// 8. Take the corresponding mapping buffer index and the result buffer index as initial parameters, and recurse to the end of all corresponding mappings available in the buffer.
int MovingObjectRemoval::recurseFindClusterChain(int col,int track)
{
  /*takes the correspondence map buffer index and result buffer index as initial parameter
  and recurses till the end of all the correspondence maps available in the buffer. cluster chain
  information is obtained from the correspondence map buffer 'corrs_vec'. consistency in the 
  cluster chain is known using the result buffer 'res_vec'. returns -1 if consistency fails
  or else returns the index of the moving cluster in the cluster collection 'clusters' in
  the latest pointcloud frame 'cb'*/

  if(col == corrs_vec.size())
  {
    /*break condition for the recursion. return the index of the moving cluster*/
    return track;
  }

  for(int j=0;j<corrs_vec[col]->size();j++)
  {
    /*search all the unit correspondeces within the correspondence map in 'col' index of the buffer*/
    
    if((*corrs_vec[col])[j].index_query == track)
    {
      /*correspondence map should have the key 'track'*/

      if(res_vec[col+1][(*corrs_vec[col])[j].index_match] == true)
      {
        /*the mapping index must have a true positive value in the result buffer
The map index must have a true value */ in the result buffer

        return recurseFindClusterChain(col+1,(*corrs_vec[col])[j].index_match);
        /*if both key and mapped index have true positive value then move for the next correspondence*/
      }
      else
      {
        return -1;
      }
    }
  }
  return -1;
}

// 9. Function to check and push moving centroids to confirmed moving cluster vectors
void MovingObjectRemoval::pushCentroid(pcl::PointXYZ pt)
{
  /*function to check and push the moving centroid to the confirmed moving cluster vector*/

	for(int i=0;i<mo_vec.size();i++)
	{
    /*check if the moving centroid has already been added to the 'mo_vec' previously*/
		double dist = sqrt(pow(pt.x-mo_vec[i].centroid.x,2)+pow(pt.y-mo_vec[i].centroid.y,2)+pow(pt.z-mo_vec[i].centroid.z,2));
		if(dist<catch_up_distance)
		{
      /*if found a centroid close to the new moving centroid then return, as no additional*/
			return;
		}
	}

	MovingObjectCentroid moc(pt,static_confidence);//static_confidence=3
  /*assign static confidence to 'moc' that determines it's persistance in 'mo_vec'*/

	mo_vec.push_back(moc);
  /*if not present then add the new cluster centroid to the 'mo_vec'*/
}

// 7. Get the new correspondence graph and result vector after the detection step, and update the buffer
void MovingObjectRemoval::checkMovingClusterChain(pcl::CorrespondencesPtr mp,std::vector<bool> &res_ca,
std::vector<bool> &res_cb)
{
  /*gets new correspondence map and result vector after the detection step and updates the
  buffers. it checks for new moving clusters and adds them to the 'mo_vec'*/

  corrs_vec.push_back(mp);
  if(res_vec.size()==0)
  {
  //   res_vec.pop_back(); //deletes the top most result in the buffer
    res_vec.push_back(res_ca);
  }
  // res_vec.push_back(res_ca); //updates the buffer with the latest result
  res_vec.push_back(res_cb);
  // std::cout<<mo_vec.size()<<" "<<corrs_vec.size()<<" "<<res_vec.size()<<std::endl;
  if(res_vec.size() >= moving_confidence)//moving_confidence=4
  {
    for(int i=0;i<res_vec[0].size();i++)
    {
      if(res_vec[0][i] == true)
      {
        /*look to the historical data in the result buffer and check the clusters with true positive
        value in the first result vector within the buffer*/

      int found_moving_index = recurseFindClusterChain(0,i);
        /*run the recursive test to find a potential cluster chain form the buffers
Run a recursive test to find potential cluster chains from the buffer */

        if(found_moving_index != -1)
        {
          /*if found push the confirmed moving centroid into 'mo_vec'*/
          pushCentroid(cb->centroid_collection->points[found_moving_index]);
        }
      }
    }
    corrs_vec.pop_front(); //delete old values from the buffer
    res_vec.pop_front();
  }
}

int counta = 0;
// 1. Receive synchronous incoming data and run detection methods
void MovingObjectRemoval::pushRawCloudAndPose(pcl::PCLPointCloud2 &in_cloud,geometry_msgs::Pose pose)
{
  /*recieves the synchronized incoming data and runs detection methods*/

  ca = cb; //update previous frame with the current frame
  cb.reset(new MovingObjectDetectionCloud(gp_limit,gp_leaf,bin_gap,min_cluster_size,max_cluster_size)); 
  // release the original space default delete, cb clear

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
  

  cb->groundPlaneRemoval(trim_x,trim_y,trim_z); //ground plane removal (hard coded)
  //cb->groundPlaneRemoval(trim_x,trim_y); //groud plane removal (voxel covariance)
  gpr.publish(cb->output_rgp);

  cb->computeClusters(ec_distance_threshold,"single_cluster"); 
  /*compute clusters within the lastet pointcloud*/

  showDistance(cb->centroid_collection);
  // Display the distance between cluster centers
  

 // output all clusters
  float rd02=0,gd02=1.0,bd02=0;
  for (int i = 0; i < cb->clusters.size(); i++)
  {
		//marker2_pub.publish(mark_cluster(cb->clusters[i],i,debug_fid,"All_box",rd02,gd02,bd02));
    marker2_pub.publish(mark_cluster2(cb->clusters[i],i,debug_fid,1));
  }

  cb->init = true; //confirm the frame for detection

  if(ca->init  == true && cb->init == true)
  {
    tf::Transform t = (cb->ps).inverseTimes(ca->ps); 
    /*calculate transformation matrix between previous and current pose. 't' transforms a point
    from the previous pose to the current pose*/


    pcl::PointCloud<pcl::PointXYZ> temp = *ca->centroid_collection;
	  pcl_ros::transformPointCloud(temp,*ca->centroid_collection,t);
    /*transform the previous centroid collection with respect to 't'*/

	for(int i=0;i<ca->clusters.size();i++)
	{
    /*transform the clusters in the collection vector of the previous frame with respect to 't'*/

		pcl::PointCloud<pcl::PointXYZI> temp;
		temp = *ca->clusters[i];
	  pcl_ros::transformPointCloud(temp,*ca->clusters[i],t);
	}
  // So far, the clustering point cloud and centroid coordinates of the previous frame point cloud stored in ca use pose information to eliminate the influence of the vehicle's own movement
  #ifdef VISUALIZE //visualize the cluster collection if VISUALIZE flag is defined
	pcl::toPCLPointCloud2(*cb->cluster_collection,in_cloud);
	pcl_conversions::fromPCL(in_cloud, output);
	output.header.frame_id = debug_fid;
	debug_pub.publish(output);
  #endif
  		
	pcl::CorrespondencesPtr mp(new pcl::Correspondences()); 
  /*correspondence map between the cluster centroids of previous and current frame*/
	  	
	// cluster correspondence methods (Global) clustering corresponding method (global)
	mth->calculateCorrespondenceCentroid(ca->clusters,cb->clusters,ca->centroid_collection,cb->centroid_collection,mp,0.1);
	/*calculate euclidian correspondence and apply the voulme constraint*/

	// moving object detection methods (Local) moving object detection method (local)
  std::vector<double> param_vec;
  if(method_choice==1)
	{
    param_vec = mth->getPointDistanceEstimateVector(ca->clusters,cb->clusters,mp);

	}
  else if(method_choice==2)
  {
    param_vec = mth->getClusterPointcloudChangeVector(ca->clusters,cb->clusters,mp,0.1);
    // The vector param_vec stores the number of new points in the corresponding cluster point cloud relative to the previous frame
  }
  /*determine the movement scores for the corresponding clusters*/
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
    /*assign the boolean results acording to thresholds. true for moving and false for static cluster*/
	}
	// std::cout<<ct<<std::endl;
  // display speed
  showV(ca->centroid_collection,cb->centroid_collection,mp,cb->detection_results);

	checkMovingClusterChain(mp,ca->detection_results,cb->detection_results);
  /*submit the results to update the buffers*/
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

// 10 Remove static clustering objects from mo_ec, remove moving dynamic objects from the latest point cloud, output filtered point cloud
bool MovingObjectRemoval::filterCloud(pcl::PCLPointCloud2 &out_cloud,std::string f_id)
{
  /*removes the moving objects from the latest pointcloud and puts the filtered cloud in 'output'.
  removes the static cluster centroids from 'mo_vec'*/

	xyz_tree.setInputCloud(cb->centroid_collection); 
  /*use kdtree for searching the moving cluster centroid within 'centroid_collection' of the
  latest frame*/

	float rd=0.8,gd=0.1,bd=0.4;int id = 1; //colour variables for visualizing red bounding box
  float rd01=0,gd01=1.0,bd01=0;
  pcl::PointIndicesPtr moving_points(new pcl::PointIndices);
  /*stores the indices of the points belonging to the moving clusters within 'cloud'*/

  // static int ct=0;
  // ct+=mo_vec.size();
  // std::cout<<ct<<std::endl;
  std::vector<int> movingclusterid;
	for(int i=0;i<mo_vec.size();i++)
	{
    /*iterate through all the moving cluster centroids*/

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);
		if(xyz_tree.nearestKSearch(mo_vec[i].centroid, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
      /*search for the actual centroid in the centroid collection of the latest frame*/

      #ifdef VISUALIZE //visualize bounding box to show the moving cluster if VISUALIZE flag is defined
			marker_pub.publish(mark_cluster2(cb->clusters[pointIdxNKNSearch[0]],id++,debug_fid,2));
			#endif
      //---------------------------------------------------------------------
      movingclusterid.push_back(pointIdxNKNSearch[0]);
      //---------------------------------------------------------------------
      
      for(int j=0;j<cb->cluster_indices[pointIdxNKNSearch[0]].indices.size();j++)
      {
        /*add the indices of the moving clusters in 'cloud' to 'moving_points'*/
        moving_points->indices.push_back(cb->cluster_indices[pointIdxNKNSearch[0]].indices[j]);
      }

			if(cb->detection_results[pointIdxNKNSearch[0]] == false || pointNKNSquaredDistance[0]>leave_off_distance)
			{
        /*decrease the moving confidence if the cluster is found to be static in the latest results or
        if the cluster dosen't appear in the current frame*/

				if(mo_vec[i].decreaseConfidence())
				{
          /*remove the moving centroid from 'mo_vec' if the confidence reduces to 0*/
					mo_vec.erase(mo_vec.begin()+i);
					i--;
				}
			}
			else
			{
				mo_vec[i].centroid = cb->centroid_collection->points[pointIdxNKNSearch[0]];
        /*update the moving centroid with the latest centroid of the moving cluster*/

				mo_vec[i].increaseConfidence(); //increase the moving confidence
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
  /*extract the moving clusters from 'cloud' and assign the filtered cloud to 'f_cloud'*/

  for(int i=0;i<cb->gp_indices->size();i++)
  {
    f_cloud->points.push_back(cb->raw_cloud->points[cb->gp_indices->at(i)]);
  }
  f_cloud->width = f_cloud->points.size();
  f_cloud->height = 1;
  f_cloud->is_dense = true;
  /*merge the ground plane to the filtered cloud
Merge ground plane to filter cloud */

  pcl::toPCLPointCloud2(*f_cloud,out_cloud);
  pcl_conversions::fromPCL(out_cloud, output);
  output.header.frame_id = f_id;
  /*assign the final filtered cloud to the 'output'*/

  return true; //confirm that a new filtered cloud is available
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

// Display the cluster center distance
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
    Marker_i.scale.z=0.3;// text size
    Marker_i.color.b=25;
    Marker_i.color.g=0;
    Marker_i.color.r=25;// text color
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

// display speed
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
    Marker_i.scale.z=0.3;// text size
    Marker_i.color.b=25;
    Marker_i.color.g=0;
    Marker_i.color.r=25;// text color
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
    Marker_j.scale.z=0.3;// size
    Marker_j.color.b=0;
    Marker_j.color.g=0;
    Marker_j.color.r=25;// color
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

// Test: Display polygon
/* void MovingObjectRemoval::showPolygon(){
  geometry_msg::PolygonStamped myPolygon;
  geometry_msgs::Point32 point;
  myPolygon.header.frame_id = debug_fid;
  
} */

// 0.1 set variable
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