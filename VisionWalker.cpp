#include "VisionWalker.h"

#include <iostream>
#include <functional>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/time.h>
#include <pcl/conversions.h>



pcl::PCLPointCloud2::Ptr VisionWalker::createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter)
{
    pcl::PCLPointCloud2::Ptr filteredCloud (new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloudToFilter);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisionWalker::runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToFilter, char *field, double min, double max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(cloudToFilter);
    passThroughFilter.setFilterFieldName(field);
    passThroughFilter.setFilterLimits(min, max);
    passThroughFilter.filter(*filteredCloud);
    return filteredCloud;
}

void VisionWalker::process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if(!viewer->wasStopped())
    {
        const pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughCloud = runPassThroughFilter(cloud, "z", 0.4, 4.0);
        pcl::PCLPointCloud2::Ptr passThroughCloud2;
        pcl::toPCLPointCloud2(*passThroughCloud, *passThroughCloud2);
        const pcl::PCLPointCloud2::Ptr voxelCloud2 = createVoxelGrid(passThroughCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*voxelCloud2, *voxelCloud);
        viewer->showCloud(voxelCloud);
    }
    
}

void VisionWalker::run()
{
    pcl::Grabber *knightsWhoGrabNi = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&)> shrubbery = 
        boost::bind(&VisionWalker::process, this, _1);

    knightsWhoGrabNi->registerCallback(shrubbery);
    knightsWhoGrabNi->start();

    while(!viewer->wasStopped())
    {
        boost::this_thread::sleep(boost::posix_time::seconds(1));
    }

    knightsWhoGrabNi->stop();
}

int main(int argc, char const *argv[])
{
    VisionWalker vw;
    vw.run();
    return 0;
}
