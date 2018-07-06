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
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>

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

bool VisionWalker::segmentFloorPlane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
{
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(RANSAC_DISTANCE_THRESHOLD);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    segmentation.setAxis(axis);
    segmentation.setEpsAngle(pcl::deg2rad(ANGLE_THRESHOLD));
    segmentation.setInputCloud(cloudToSegment);
    segmentation.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        return false;
    }
    else
    {
        return true;
    }
    
}

std::vector<pcl::PointIndices> VisionWalker::segmentWallPlanes(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment)
{
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(RANSAC_DISTANCE_THRESHOLD);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0);
    segmentation.setAxis(axis);
    segmentation.setEpsAngle(pcl::deg2rad(ANGLE_THRESHOLD));
    std::vector<pcl::PointIndices> inlier_vector;
    bool finishedSegmentation = false;
    segmentation.setInputCloud(cloudToSegment);
    do
    {
        pcl::ModelCoefficients::Ptr wall_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr wall_inliers(new pcl::PointIndices);
        segmentation.segment(*inliers, *coefficients);
        if(inliers->indices.size() != 0)
        {
            inlier_vector.push_back(wall_inliers);
        }
    } while (!finishedSegmentation);
    return inlier_vector;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisionWalker::extractPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractCloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*extractCloud);
    return extractCloud;
}

    void VisionWalker::process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if(!viewer->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughCloud = runPassThroughFilter(cloud, "z", MINIMUM_Z, MAXIMUM_Z);
        passThroughCloud = runPassThroughFilter(cloud, "x", MINIMUM_X, MAXIMUM_X);
        pcl::PCLPointCloud2::Ptr passThroughCloud2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*passThroughCloud, *passThroughCloud2);
        pcl::PCLPointCloud2::Ptr voxelCloud2 = createVoxelGrid(passThroughCloud2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*voxelCloud2, *voxelCloud);
        pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr floor_inliers(new pcl::PointIndices);
        bool isFloor = segmentFloorPlane(voxelCloud, floor_coefficients, floor_inliers);
        if(isFloor)
        {
            voxelCloud = extractPoints(voxelCloud, floor_inliers);
        }
        std::vector<pcl::PointIndices> inlier_vector = segmentWallPlanes(voxelCloud);
        for(int i = 0; i < inlier_vector.size(); i++)
        {
            voxelCloud = extractPoints(voxelCloud, inlier_vector);
        }
        viewer->addPointCloud(voxelCloud, "objects");
        viewer->spin();
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
