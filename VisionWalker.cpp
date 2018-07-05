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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr VisionWalker::runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudToFilter, char *field, double min, double max)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PassThrough<pcl::PointXYZRGBA> passThroughFilter;
    passThroughFilter.setInputCloud(cloudToFilter);
    passThroughFilter.setFilterFieldName(field);
    passThroughFilter.setFilterLimits(min, max);
    passThroughFilter.filter(*filteredCloud);
    return filteredCloud;
}

void VisionWalker::runPlanarSegmentation(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
{
    pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);

    segmentation.setInputCloud(cloudToSegment);
    segmentation.segment(*inliers, *coefficients);
}

void VisionWalker::process(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    if(!viewer->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr passThroughCloud = runPassThroughFilter(cloud, "z", 0.4, 4.0);
        pcl::PCLPointCloud2::Ptr passThroughCloud2(new pcl::PCLPointCloud2);
        pcl::toPCLPointCloud2(*passThroughCloud, *passThroughCloud2);
        pcl::PCLPointCloud2::Ptr voxelCloud2 = createVoxelGrid(passThroughCloud2);
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::fromPCLPointCloud2(*voxelCloud2, *voxelCloud);
        // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // runPlanarSegmentation(voxelCloud, coefficients, inliers);
        // pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr extractCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
        // extract.setInputCloud(voxelCloud);
        // extract.setIndices(inliers);
        // extract.setNegative(true);
        // extract.filter(*extractCloud);
        pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimation;
        normal_estimation.setInputCloud(voxelCloud);
        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA>());
        normal_estimation.setSearchMethod(tree);
        normal_estimation.setRadiusSearch(0.03);
        normal_estimation.compute(*normal_cloud);
        pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGBA, pcl::Normal, pcl::Label> mps;
        mps.setMinInliers(10000);
        mps.setAngularThreshold(0.017453 * 2.0);
        mps.setInputNormals(normal_cloud);
        mps.setInputCloud(voxelCloud);
        std::vector< pcl::PlanarRegion< pcl::PointT > > regions;
        mps.segmentAndRefine(regions);
        //viewer->showCloud(voxelCloud);
        viewer->showCloud(voxelCloud);
    }
    
}

void VisionWalker::run()
{
    pcl::Grabber *knightsWhoGrabNi = new pcl::OpenNIGrabber();

    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> shrubbery = 
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
