
// Includes all the required headers for clustersing*//

#include <array>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;
using namespace pcl;

const int minClusterdNum2 = 80;
const float roiM2 = 20;

class cluster_seed2
{
private:
  int clusterdPointNum;
  std::vector<int> clusterdNum;

public:
  cluster_seed2()
  {
    clusterdPointNum = 0;
  }
  void updatePointId(int j)
  {
    clusterdNum.push_back(j);
  }
  void updatePointNum()
  {
    clusterdPointNum++;
  }
  int get_clusterdPointNum()
  {
    return clusterdPointNum;
  }
  int get_clusterPointId(int k)
  {
    return clusterdNum[k];
  }
};

class DBScan
{
private:
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  std::vector<int> visited;
  float epsilon;
  int minNeighbor;
  int minSignPoint;
  int nextGroupID;
  // float resolution = 0.3f;

public:
  // pcl::KdTree<pcl::PointXYZ>::Ptr tree;

  DBScan(pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_xyz, float eps, int minNeigh, int minPnt);

  std::vector<int> find_neighbors_3d(int home);

  void clustering(int home, std::vector<int> neighbors);

  float get_distance_3d(int i1, int i2);

  void start_clustering();

  int get_numCluster();

  int getClusterID(int i)
  {
    return visited[i];
  }
};

void DBSCAN_getClusteredPoints(PointCloud<PointXYZI>::Ptr elevatedCloud, vector<cluster_seed2>& cluster_seed_,
                               int numCluster, std::vector<pcl::PointIndices>& cluster_indices, DBScan dbscan_finder);

void DBSCAN_Clustering(PointCloud<pcl::PointXYZI>::Ptr elevatedCloud, std::vector<pcl::PointIndices>& cluster_indices);
