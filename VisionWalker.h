#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/pcl_visualizer.h>

#ifndef VISION_WALKER
#define VISION_WALKER

class VisionWalker
{
    public:
    VisionWalker()
    {
        viewer = new pcl::visualization::PCLVisualizer ("PCL OpenNI Viewer");
    }

    pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter);
    pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToFilter, char *field, double min, double max);
    bool segmentFloorPlane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
    std::vector<pcl::PointIndices> segmentWallPlanes(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPoints(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers);
    void run();
    void process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    private:
    pcl::visualization::PCLVisualizer* viewer;
    static const int INLIER_THRESHOLD = 20000;
    static const float RANSAC_DISTANCE_THRESHOLD = 0.1;
    static const float MINIMUM_X = -0.4;
    static const float MAXIMUM_X = 0.4;
    static const float MINIMUM_Z = 0.4;
    static const float MAXIMUM_Z = 4.0;
    static const float ANGLE_THRESHOLD = 15.0;
};

#endif