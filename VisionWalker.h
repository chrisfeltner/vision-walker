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
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToFilter, char *field, double min, double max);
    void runPlanarSegmentation(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
    void run();
    void process(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

    private:
    pcl::visualization::CloudViewer* viewer;
};

#endif