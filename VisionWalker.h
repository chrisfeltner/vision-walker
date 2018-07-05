#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>

#ifndef VISION_WALKER
#define VISION_WALKER

class VisionWalker
{
    public:
    VisionWalker()
    {
        viewer = new pcl::visualization::CloudViewer("PCL OpenNI Viewer");
    }

    pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter);
    pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToFilter, char *field, double min, double max);
    void runPlanarSegmentation(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers);
    void run();
    void process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    private:
    pcl::visualization::CloudViewer* viewer;
    static const int INLIER_THRESHOLD = 20000;
    static const float RANSAC_DISTANCE_THRESHOLD = 0.03;
};

#endif