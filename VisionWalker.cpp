#include "VisionWalker.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>

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
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(cloudToFilter);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PCLPointCloud2::Ptr runPassThroughFilter(pcl::PCLPointCloud2::Ptr cloudToFilter = NULL,
    char *field, double min, double max)
{
    if(cloudToFilter == NULL)
    {
        return NULL;
    }

    pcl::PCLPointCloud2::Ptr filteredCloud = pcl::PCLPointCloud2();
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(cloudToFilter);
    passThroughFilter.setFilterFieldName(field);
    passThroughFilter.setFilterLimits(min, max);
    passThroughFilter.filter(*filteredCloud);
    return filteredCloud;
}