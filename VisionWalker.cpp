#include <VisionWalker.h>
#include <pcl/filters/voxel_grid.h>


VisionWalker::VisionWalker()
{

}

pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter = NULL)
{
    if(cloudToFilter == NULL)
    {
        return NULL;
    }

    pcl::PCLPointCloud2::Ptr filteredCloud = pcl::PCLPointCloud2();
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloudToFilter);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}