
#include "component_clustering.h"

int kernelSize = 3;

// 初始化网格Grid状态
void mapCartesianGrid(PointCloud<PointXYZI>::Ptr elevatedCloud,
                            array<array<int, numGrid>, numGrid> & cartesianData){
    array<array<int, numGrid>, numGrid> gridNum{};  
    //  gridNum 此数组专门统计落在每个grid的点云数
    for(int cellX = 0; cellX < numGrid; cellX++){   
        for(int cellY = 0; cellY < numGrid; cellY++){   
            gridNum[cellX][cellY] = 0; 
        }
    }
    // elevatedCloud 映射到笛卡尔坐标系 
    for(int i = 0; i < elevatedCloud->size(); i++){  
        float x = elevatedCloud->points[i].x;   
        float y = elevatedCloud->points[i].y;
        float xC = x+roiM/2;   
        float yC = y+roiM/2; 
        // exclude outside roi points  排除外部roi points  x,y属于(-25, 25)下面才继续执行
        if(xC < 0 || xC >= roiM || yC < 0 || yC >=roiM) continue; 
        int xI = floor(numGrid*xC/roiM);   //   floor(x)返回的是小于或等于x的最大整数
        int yI = floor(numGrid*yC/roiM);   
        gridNum[xI][yI] = gridNum[xI][yI] + 1;  // 统计落在这个grid的点数
    }
    // 将x，y位置的单个单元格选作中心单元格，并且clusterID计数器加1。
    // 然后所有相邻的相邻像元（即x-1，y  + 1，x，y +1，x +1，y +1 x -1，y，x +1，y，x -1，y -1，x，检查y − 1，x + 1，y +  1）的占用状态，并用当前集群ID标记。
    // 对m×n网格中的每个x，y重复此过程，直到为所有非空群集分配了ID。
    for(int xI = 0; xI < numGrid; xI++){  
        for(int yI = 0; yI < numGrid; yI++){
            if(gridNum[xI][yI] > 2){  
                cartesianData[xI][yI] = -1;   // 网格分配有2种初始状态，分别为空（0），已占用（-1）和已分配
                if(xI == 0)
                {
                    if(yI == 0)
                    {
                        cartesianData[xI+1][yI] = -1;  // 角相邻的3个相邻像元
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI < numGrid - 1)
                    {
                        cartesianData[xI][yI-1] = -1;  // 边有5个相邻点
                        cartesianData[xI][yI+1] = -1;
                        cartesianData[xI+1][yI-1] = -1;
                        cartesianData[xI+1][yI] = -1;
                        cartesianData[xI+1][yI+1] = -1;
                    }
                    else if(yI == numGrid - 1)  // 角相邻的3个相邻像元
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
                    else if(yI < numGrid - 1)  // 一般情况四周有8个相邻点
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

//聚类    图搜索  
void search(array<array<int, numGrid>, numGrid> & cartesianData, int clusterId, int cellX, int cellY){   
    cartesianData[cellX][cellY] = clusterId; // 赋值，将周边赋值同样的clusterId
    int mean = kernelSize/2;   // kernelSize = 3;  mean  = 1 
    for (int kX = 0; kX < kernelSize; kX++){   // kernelSize = 3;循环3次
        int kXI = kX-mean; //    0， -1 ， 1 
        if((cellX + kXI) < 0 || (cellX + kXI) >= numGrid) continue;   
        for( int kY = 0; kY < kernelSize;kY++){
            int kYI = kY-mean; 
            if((cellY + kYI) < 0 || (cellY + kYI) >= numGrid) continue;

            if(cartesianData[cellX + kXI][cellY + kYI] == -1){
                search(cartesianData, clusterId, cellX +kXI, cellY + kYI);  // 循环搜索
            }

        }
    }
}

//  对m×n网格中的每个x，y重复此过程，直到为所有非空cluster分配了ID。
void findComponent(array<array<int, numGrid>, numGrid> & cartesianData,  int &clusterId){
    for(int cellX = 0; cellX < numGrid; cellX++){  
        for(int cellY = 0; cellY < numGrid; cellY++){
            if(cartesianData[cellX][cellY] == -1){   
                clusterId ++;    
                search(cartesianData, clusterId, cellX, cellY);  // 对每一个点进行搜索  
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

        int clusterNum = cartesianData[xI][yI]; //1 ~ numCluster   初始聚类ID数量numCluster
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
    mapCartesianGrid(elevatedCloud, cartesianData); // 设置网格Grid状态   网格数组：cartesianData
    findComponent(cartesianData, numCluster);  //  聚类ID分配
    //cluster_indices.reserve(numCluster);
    vector<cluster_seed>cluster_seed_(numCluster);
    getClusteredPoints(elevatedCloud, cartesianData, cluster_seed_, numCluster,cluster_indices);  // 依据聚类ID，将每个点存入对应的聚类 cluster_indices
}
