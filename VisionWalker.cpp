#include "VisionWalker.h"

#include <iostream>
#include <functional>
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <curses.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>

pcl::PCLPointCloud2::Ptr VisionWalker::createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter)
{
    pcl::PCLPointCloud2::Ptr filteredCloud (new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloudToFilter);
    voxelFilter.setLeafSize(0.01f, 0.01f, 0.01f);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisionWalker::runPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFilter, const char *field, double min, double max)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> passThroughFilter;
    passThroughFilter.setInputCloud(cloudToFilter);
    passThroughFilter.setFilterFieldName(field);
    passThroughFilter.setFilterLimits(min, max);
    passThroughFilter.filter(*filteredCloud);
    return filteredCloud;
}

bool VisionWalker::segmentFloorPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
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

std::vector<pcl::PointIndices::Ptr> VisionWalker::segmentWallPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSegment)
{
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(RANSAC_DISTANCE_THRESHOLD);
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0);
    segmentation.setAxis(axis);
    segmentation.setEpsAngle(pcl::deg2rad(ANGLE_THRESHOLD));
    std::vector<pcl::PointIndices::Ptr> inlier_vector;
    bool finishedSegmentation = false;
    segmentation.setInputCloud(cloudToSegment);
    do
    {
        pcl::ModelCoefficients::Ptr wall_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr wall_inliers(new pcl::PointIndices);
        segmentation.segment(*wall_inliers, *wall_coefficients);
        if(wall_inliers->indices.size() != 0)
        {
            inlier_vector.push_back(wall_inliers);
        }
        finishedSegmentation = wall_inliers->indices.size() < INLIER_THRESHOLD;
    } while (!finishedSegmentation);
    return inlier_vector;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr VisionWalker::extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractCloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*extractCloud);
    return extractCloud;
}

double VisionWalker::findMinimumDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double min = std::numeric_limits<double>::max();

    for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin(); it != cloud->end(); it++)
    {
        if(it->z < min)
        {
            min = it->z;
        }
    }

    return min;
}

void VisionWalker::process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    if(!viewer->wasStopped())
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZ>);
        *myCloud = *cloud;
        if(PASS_THROUGH_FILTER)
        {
            myCloud = runPassThroughFilter(myCloud, "z", MINIMUM_Z, MAXIMUM_Z);
            myCloud = runPassThroughFilter(myCloud, "x", MINIMUM_X, MAXIMUM_X);
        }

        if(STATISTICAL_FILTER)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(myCloud);
            sor.setMeanK(MEAN_K);
            sor.setStddevMulThresh(STANDARD_DEVIATION);
            sor.filter(*myCloud);
        }

        if(VOXEL_FILTER)
        {
            pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2(*myCloud, *cloud2);
            cloud2 = createVoxelGrid(cloud2);
            pcl::fromPCLPointCloud2(*cloud2, *myCloud);
        }

        if(FLOOR_SEGMENTATION)
        {
            pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr floor_inliers(new pcl::PointIndices);
            bool isFloor = segmentFloorPlane(myCloud, floor_coefficients, floor_inliers);

            if (isFloor)
            {
                myCloud = extractPoints(myCloud, floor_inliers);
            }
            else
            {
                //beep
            }
        }
        
        if(myCloud->size() >= OBSTACLE_SIZE_THRESHOLD)
        {
            double min = findMinimumDistance(myCloud);
            std::cout << "Minimum distance: " << min << std::endl;
        }
        else
        {
            std::cout << "No obstacle detected." << std::endl;
        }

        viewer->showCloud(myCloud);
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
