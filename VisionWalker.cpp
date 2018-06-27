#include "VisionWalker.h"

#include <iostream>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>


pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter)
{
    pcl::PCLPointCloud2::Ptr filteredCloud (new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloudToFilter);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToFilter, char *field, double min, double max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(cloudToFilter);
    passThroughFilter.setFilterFieldName(field);
    passThroughFilter.setFilterLimits(min, max);
    passThroughFilter.filter(*filteredCloud);
    return filteredCloud;
}

void process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud = runPassThroughFilter(cloud, "z", 0.4, 4.0);
}

void run()
{
    pcl::Grabber *knightsWhoGrabNi = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr)> shrubbery = 
        boost::bind(&VisionWalker::process, _1);

    knightsWhoGrabNi->registerCallback(shrubbery);
    knightsWhoGrabNi->start();
}

int main(int argc, char const *argv[])
{
    return 0;
}
