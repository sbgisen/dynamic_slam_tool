#include "DBSCAN_clustering.h"

DBScan::DBScan(pcl::PointCloud<pcl::PointXYZI>::Ptr& filtered_xyz, float eps, int minNeigh, int minPnt)
{
  cloud = filtered_xyz;
  epsilon = eps;
  minNeighbor = minNeigh;
  minSignPoint = minPnt;
  nextGroupID = 1;

  visited.reserve(cloud->points.size());
  visited.insert(visited.end(), cloud->size(), 0);

  // tree->setInputCloud(cloud);
}

// float get_distance_3d(int i1, int i2);

std::vector<int> DBScan::find_neighbors_3d(int home)
{
  std::vector<int> neighbors;
  /* std::vector<int>pointIdxNKNSearch;
  std::vector<float>pointNKNSquaredDistance;
  if (tree->radiusSearch (cloud->points[home], 0.21, pointIdxNKNSearch, pointNKNSquaredDistance) >0)
  {
      for (size_t i=0; i<pointIdxNKNSearch.size (); ++i)
      {
          if (pointNKNSquaredDistance[i] < epsilon && i != home and visited[i] < 1){
              neighbors.push_back(i);
          }
      }
  }  */
  for (std::vector<int>::size_type i = 0; i != cloud->points.size(); i++)
  {
    if (get_distance_3d(i, home) < epsilon && i != home and visited[i] < 1)
    {
      neighbors.push_back(i);
    }
  }
  return neighbors;
}

float DBScan::get_distance_3d(int i1, int i2)
{
  pcl::PointXYZI num1 = cloud->points[i1];
  pcl::PointXYZI num2 = cloud->points[i2];

  float a = sqrt((num1.x - num2.x) * (num1.x - num2.x) + (num1.y - num2.y) * (num1.y - num2.y) +
                 (num1.z - num2.z) * (num1.z - num2.z));

  return sqrt((num1.x - num2.x) * (num1.x - num2.x) + (num1.y - num2.y) * (num1.y - num2.y) +
              (num1.z - num2.z) * (num1.z - num2.z));
}

void DBScan::clustering(int home, std::vector<int> neighbors)
{
  int cluster_id = visited[home];

  for (std::vector<int>::iterator neighbor = neighbors.begin(); neighbor != neighbors.end(); ++neighbor)
  {
    if (visited[*neighbor] == 0)
    {
      visited[*neighbor] = cluster_id;

      std::vector<int> sub_neighbors = find_neighbors_3d(*neighbor);
      clustering(*neighbor, sub_neighbors);
    }
  }
}

void DBScan::start_clustering()
{
  for (std::vector<int>::size_type i = 0; i != cloud->points.size(); i++)
  {
    if (visited[i] == 0)
    {
      std::vector<int> neighbors = find_neighbors_3d(i);

      if (neighbors.size() < minNeighbor)
      {
        visited[i] = -1;
      }
      else
      {
        visited[i] = nextGroupID;
        nextGroupID++;

        clustering(i, neighbors);
      }
    }
  }
}

int DBScan::get_numCluster()
{
  start_clustering();
  int max = -999;
  for (auto v : visited)
  {
    if (max < v)
      max = v;
  }
  return max;

  /* std::map<int, int> counters;

  for(int i = 0; i < visited.size(); i++){
      if(visited[i] > 0)
      counters[visited[i]]++;
  }
  std::vector<int> groups;

  for(std::map<int, int>::iterator it = counters.begin(); it != counters.end(); it++){
      groups.push_back(it->first);
  }
  for (int i = 1; i < groups.size(); i++)
  {
      int j = i - 1;
      int key = groups[i];

      while(j >= 0 && counters[groups[j]] < counters[key]){
          groups[j+1] = groups[j];
          j--;
      }
      groups[j+1] = key;
  }

  for (int i = 1; i < groups.size(); i++)
  {
      std::cout<<groups[i]<<":"<< counters[groups[i]] <<std::endl;
  } */
}

void DBSCAN_getClusteredPoints(PointCloud<PointXYZI>::Ptr elevatedCloud, vector<cluster_seed2>& cluster_seed_,
                               int numCluster, std::vector<pcl::PointIndices>& cluster_indices, DBScan dbscan_finder)
{
  for (int i = 0; i < elevatedCloud->size(); i++)
  {
    float x = elevatedCloud->points[i].x;
    float y = elevatedCloud->points[i].y;
    float z = elevatedCloud->points[i].z;

    // exclude outside roi points
    if (x < -roiM2 || x >= roiM2 || y < -roiM2 || y >= roiM2)
      continue;

    int clusterNum = dbscan_finder.getClusterID(i);
    int vectorInd = clusterNum - 1;  // 0 ~ (numCluster -1 )
    if (clusterNum > 0)
    {
      cluster_seed_[vectorInd].updatePointId(i);
      cluster_seed_[vectorInd].updatePointNum();
    }
  }
  for (int id = 0; id < numCluster - 1; id++)
  {
    if (cluster_seed_[id].get_clusterdPointNum() < minClusterdNum2)
      continue;
    pcl::PointIndices r;
    r.indices.resize(cluster_seed_[id].get_clusterdPointNum());
    for (int j = 0; j < cluster_seed_[id].get_clusterdPointNum(); j++)
      r.indices[j] = cluster_seed_[id].get_clusterPointId(j);
    cluster_indices.push_back(r);
  }
}

void DBSCAN_Clustering(PointCloud<pcl::PointXYZI>::Ptr elevatedCloud, std::vector<pcl::PointIndices>& cluster_indices)
{
  DBScan dbscan_finder(elevatedCloud, 0.21, 4, 1);
  int numCluster = dbscan_finder.get_numCluster();
  vector<cluster_seed2> cluster_seed_(numCluster);
  DBSCAN_getClusteredPoints(elevatedCloud, cluster_seed_, numCluster, cluster_indices, dbscan_finder);
}
