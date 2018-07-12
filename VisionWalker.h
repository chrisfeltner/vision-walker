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

    enum DISTANCE
    {
        WARN_0,
        WARN_1,
        WARN_2,
        WARN_3
    };

    pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter);
    pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToFilter, const char *field, double min, double max);
    bool segmentFloorPlane(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
    std::vector<pcl::PointIndices::Ptr> segmentWallPlanes(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToSegment);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointIndices::Ptr inliers);
    double findMinimumDistance(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
    void run();
    void process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    private:
    pcl::visualization::CloudViewer* viewer;
    static const int INLIER_THRESHOLD = 20000;
    static const float RANSAC_DISTANCE_THRESHOLD = 0.03;
    static const float MINIMUM_X = -0.4;
    static const float MAXIMUM_X = 0.4;
    static const float MINIMUM_Z = 0.4;
    static const float MAXIMUM_Z = 4.0;
    static const float ANGLE_THRESHOLD = 15.0;
    static const float OBSTACLE_SIZE_THRESHOLD = 1000;
    static const int MEAN_K = 50;
    static const float STANDARD_DEVIATION = 1.0;
    static const bool STATISTICAL_FILTER = true;
    static const bool VOXEL_FILTER = true;
    static const bool PASS_THROUGH_FILTER = true;
    static const bool FLOOR_SEGMENTATION = true;
};

#endif