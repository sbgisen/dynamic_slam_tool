
// Includes all the required headers for clustersing*///

#include <array>
#include <random>
#include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <vector>

#include <iostream>
#include <math.h>

using namespace std;
using namespace pcl;
using namespace cv;
using namespace Eigen;

const float GridSize = 0.2;  // grid size 0.2*0.2m
const float roiM = 30;
const int numGrid = 300;  // discretized into 100*100 grid according to xy plane
const float picScale = 900 / roiM;
const int minClusterdNum = 80;  // The minimum number of clustering points

class cluster_seed
{
private:
  int clusterdPointNum;
  std::vector<int> clusterdNum;

public:
  cluster_seed()
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

void mapCartesianGrid(PointCloud<PointXYZI>::Ptr elevatedCloud, array<array<int, numGrid>, numGrid>& cartesianData);

void search(array<array<int, numGrid>, numGrid>& cartesianData, int clusterId, int cellX, int cellY);

void findComponent(array<array<int, numGrid>, numGrid>& cartesianData, int& clusterId);

void componentClustering(PointCloud<pcl::PointXYZI>::Ptr elevatedCloud, std::vector<pcl::PointIndices>& cluster_indices);

void getClusteredPoints(PointCloud<PointXYZI>::Ptr elevatedCloud, array<array<int, numGrid>, numGrid> cartesianData,
                        vector<cluster_seed>& cluster_seed_, int numCluster,
                        std::vector<pcl::PointIndices>& cluster_indices);
