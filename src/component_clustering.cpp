
#include "component_clustering.h"

int kernelSize = 3;

// Initialize the state of the grid Grid
void mapCartesianGrid(PointCloud<PointXYZI>::Ptr elevatedCloud,
                            array<array<int, numGrid>, numGrid> & cartesianData){
    array<array<int, numGrid>, numGrid> gridNum{};  
    // gridNum This array specifically counts the number of point clouds falling on each grid
    for(int cellX = 0; cellX < numGrid; cellX++){   
        for(int cellY = 0; cellY < numGrid; cellY++){   
            gridNum[cellX][cellY] = 0; 
        }
    }
    // elevatedCloud maps to Cartesian coordinate system
    for(int i = 0; i < elevatedCloud->size(); i++){  
        float x = elevatedCloud->points[i].x;   
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM/2;   
        float yC = y+roiM/2; 
        // exclude outside roi points Exclude external roi points x, y belong to (-25, 25) before proceeding
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue; 
        int xI = floor(numGrid*xC/roiM);   // floor(x) returns the largest integer less than or equal to x
        int yI = floor(numGrid*yC/roiM);   
        gridNum[xI][yI] = gridNum[xI][yI] + 1;  // Count the points that fall on this grid
    }
    // Select a single cell at x, y position as the center cell, and increment the clusterID counter by 1.
    // Then all adjacent neighbor cells (i.e. x-1,y+1,x,y+1,x+1,y+1 x-1,y,x+1,y,x - 1, y -1, x, check the occupancy status of y − 1, x + 1, y + 1) and mark it with the current cluster ID.
    // Repeat this process for each x,y in the mxn grid until all non-empty clusters have been assigned IDs.
    for(int xI = 0; xI < numGrid; xI++){  
        for(int yI = 0; yI < numGrid; yI++){
            if(gridNum[xI][yI] > 2){  
                cartesianData[xI][yI] = -1;   // Grid allocation has 2 initial states, which are empty (0), occupied (-1) and allocated
                if(xI == 0)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI+1][yI] = -1;  // 3 adjacent cells adjacent to the corner
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI][yI-1] = -1;  // side has 5 adjacent points
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI == numGrid - 1)  // 3 adjacent cells adjacent to the corner
                    {
                        cartesianData[xI][yI-1] = -1; 
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;    
                    }
                }
                else if(xI < numGrid - 1)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;                
                    }
                    else if(yI < numGrid - 1)  // Generally, there are 8 adjacent points around
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI-1] = -1;
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;                  
                    }
                    else if(yI == numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI][yI-1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;                 
                    } 
                }
                else if(xI == numGrid - 1)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI+1] = -1;
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI-1][yI+1] = -1;
                        cartesianData[xI][yI-1] = -1;
                        cartesianData[xI][yI+1] = -1;
                    }
                    else if(yI == numGrid - 1)
                    {
                        cartesianData[xI-1][yI-1] = -1;
                        cartesianData[xI-1][yI] = -1;
                        cartesianData[xI][yI-1] = -1;    
                    }            
                }
            }
        }
    }
}

// cluster graph search
void search(array<array<int, numGrid>, numGrid> & cartesianData, int clusterId, int cellX, int cellY){   
    cartesianData[cellX][cellY] = clusterId; // Assignment, assign the same clusterId to the surrounding
    int mean = kernelSize/2;   // kernelSize = 3;  mean  = 1 
    for (int kX = 0; kX < kernelSize; kX++){   // kernelSize = 3; Loop 3 times
        int kXI = kX-mean; // 0, -1, 1
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid) continue;   
        for( int kY = 0; kY < kernelSize;kY++){
            int kYI = kY-mean; 
            if((cellY + kYI) < 0 || (cellY + kYI) >= numGrid) continue;

            if(cartesianData[cellX + kXI][cellY + kYI] == -1){
                search(cartesianData, clusterId, cellX +kXI, cellY + kYI);  // loop search
            }

        }
    }
}

// Repeat this process for each x, y in the m×n grid until all non-empty clusters have been assigned IDs.
void findComponent(array<array<int, numGrid>, numGrid> & cartesianData,  int &clusterId){
    for(int cellX = 0; cellX < numGrid; cellX++){  
        for(int cellY = 0; cellY < numGrid; cellY++){
            if(cartesianData[cellX][cellY] == -1){   
                clusterId ++;    
                search(cartesianData, clusterId, cellX, cellY);  // Search for each point
            }
        }
    }
}

void getClusteredPoints(PointCloud<PointXYZI>::Ptr elevatedCloud,
                        array<array<int, numGrid>, numGrid> cartesianData,
                        vector<cluster_seed>&cluster_seed_, int numCluster,
                        std::vector<pcl::PointIndices> &cluster_indices) {
    for (int i = 0; i < elevatedCloud->size(); i++) {
        float x = elevatedCloud->points[i].x;
        float y = elevatedCloud->points[i].y;
        float z = elevatedCloud->points[i].z;
        float xC = x + roiM / 2;
        float yC = y + roiM / 2;
        // exclude outside roi points
        if (xC < 0 || xC >= roiM || yC < 0 || yC >= roiM) continue;
        int xI = floor(numGrid * xC / roiM);
        int yI = floor(numGrid * yC / roiM);

        int clusterNum = cartesianData[xI][yI]; // 1 ~ numCluster initial cluster ID number numCluster
        int vectorInd = clusterNum - 1; //0 ~ (numCluster -1 )
        if (clusterNum != 0) {
            cluster_seed_[vectorInd].updatePointId(i);
            cluster_seed_[vectorInd].updatePointNum();
        }
    }
    for(int id=0;id<numCluster-1;id++){
        if(cluster_seed_[id].get_clusterdPointNum() < minClusterdNum) continue;
        pcl::PointIndices r;
        r.indices.resize (cluster_seed_[id].get_clusterdPointNum());
        for (int j = 0; j < cluster_seed_[id].get_clusterdPointNum(); j++)
        r.indices[j] = cluster_seed_[id].get_clusterPointId(j);
        //r.header =elevatedCloud.header;
        cluster_indices.push_back (r);
    } 

}


void componentClustering(PointCloud<pcl::PointXYZI>::Ptr elevatedCloud,
                        std::vector<pcl::PointIndices> &cluster_indices){
    array<array<int, numGrid>, numGrid> cartesianData{};
    int numCluster=0;
    mapCartesianGrid(elevatedCloud, cartesianData); // Set grid Grid state Grid array: cartesianData
    findComponent(cartesianData, numCluster);  // Cluster ID assignment
    //cluster_indices.reserve(numCluster);
    vector<cluster_seed>cluster_seed_(numCluster);
    getClusteredPoints(elevatedCloud, cartesianData, cluster_seed_, numCluster,cluster_indices);  // According to the cluster ID, store each point in the corresponding cluster cluster_indices
}
