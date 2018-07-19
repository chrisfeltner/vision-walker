#include "VisionWalker.h"
#include <iostream>
#include <functional>
#include <boost/bind.hpp>
#include <boost/function.hpp>
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

// createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter)
// \brief Downsamples the given PCLPointCloud2. A Voxel Grid is created with cube size of LEAF_SIZE.
//        Note that this function works with PCLPointCloud2
// \param cloudToFilter A PCLPointCloud2 pointer
// \return A pointer to a PCLPointCloud2 which is the filtered cloud
pcl::PCLPointCloud2::Ptr VisionWalker::createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter)
{
    pcl::PCLPointCloud2::Ptr filteredCloud (new pcl::PCLPointCloud2());
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxelFilter;
    voxelFilter.setInputCloud(cloudToFilter);
    voxelFilter.setLeafSize(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE);
    voxelFilter.filter(*filteredCloud);
    return filteredCloud;
}

// runPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFilter, const char* field, double min, double max)
// \brief Runs a Pass Through filter. A PT Filter removes any points with a value greater than or less than the 
//        specified minimum and maximum values
// \param cloudToFilter A pointer to a point cloud with PointXYZ points
// \param field A string (char array) indicating the axis to run the filter on ("x" "y" or "z")
// \param min A minimum pass through value
// \param max A minimum pass through value
// \return A pointer to a PointCloud<pcl::PointXYZ> which is the filtered cloud
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

// segmentFloorPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers)
// \brief Segments the most significant plane parallel to the z-axis. This is assumed to be the floor.
// \param cloudToSegment A point cloud pointer with PointXYZ points
// \param coefficients A pointer to a ModelCoefficients object. This object will be set with the coefficients
//          of the resulting plane model. It is not necessary to know these before running the function.
//          The equation of a plane is ax + by + cz = d. This object will be set with the values of a, b, c, and d
//          after running RANSAC.
// \param inliers A pointer to a PointIndices object. This contains the indices of the inliers to the model.
// \return bool if there is a floor found, return true. Otherwise false.
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

// This function is not used, but is left for reference. Segments all wall planes found ... or at least it tries to.
// It works in much the same way as segmentFloorPlane
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

// extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers)
// \brief Extracts the points in the floor plane. Returns the negative of these points, meaning that it returns
//          the outliers to the floor plane.
// \param cloud A pointer to a point cloud with points PointXYZ to extract the points from
// \param inliers A pointer to a PointIndices object which has the indices of plane inliers
// \return pcl::PointCloud<pcl::PointXYZ>::Ptr A pointer to a point cloud of plane outliers
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

// findMinimumDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
// \brief Iterates through all points in a cloud and returns the minimum z value. Returns
//          std::numeric_limits<double>::max() if there are no points in the cloud.
// \param cloud A pointer to a PointCloud with PointXYZ
// \return double Maximum z value in cloud
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
// process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
// \brief This function should be passed as a pointer to OpenNiGrabber. It is called each time a 
//          point cloud is grabbed from the Kinect by OpenNI.
// \param cloud A const cloud pointer passed in by OpenNIGrabber
void VisionWalker::process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud)
{
    //Stop execution if there is no viewer process
    if(!viewer->wasStopped())
    {
        //Convert const pointer to pointer
        pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud(new pcl::PointCloud<pcl::PointXYZ>);
        *myCloud = *cloud;

        //Run pass through filter
        if(PASS_THROUGH_FILTER)
        {
            myCloud = runPassThroughFilter(myCloud, "z", MINIMUM_Z, MAXIMUM_Z);
            myCloud = runPassThroughFilter(myCloud, "x", MINIMUM_X, MAXIMUM_X);
        }

        //Run voxel filter. Handle cloud conversions.
        if(VOXEL_FILTER)
        {
            pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
            pcl::toPCLPointCloud2(*myCloud, *cloud2);
            cloud2 = createVoxelGrid(cloud2);
            pcl::fromPCLPointCloud2(*cloud2, *myCloud);
        }

        //Run statistical filter.
        if(STATISTICAL_FILTER)
        {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(myCloud);
            sor.setMeanK(MEAN_K);
            sor.setStddevMulThresh(STANDARD_DEVIATION);
            sor.filter(*myCloud);
        }

        //Segment floor.
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
                std::cout << "No floor detected! Immediate obstacle!" << std::endl;
            }
        }

        //If the number of points in the cloud is greater than threshold, find minimum z       
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

// run()
// \brief Set up OpenNIGrabber and initiate grabber thread. Stop if viewer is closed.
//          This function is a little tricker because of the callbacks. See this tutorial for
//          help: http://pointclouds.org/documentation/tutorials/openni_grabber.php
void VisionWalker::run()
{
    //For the uninitiated: https://youtu.be/zIV4poUZAQo
    pcl::Grabber *knightsWhoGrabNi = new pcl::OpenNIGrabber();

    //create a boost::function to pass in as a callback to be called each time a new point cloud is grabbed.
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
