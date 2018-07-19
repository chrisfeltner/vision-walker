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
    pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFilter, const char *field, double min, double max);
    bool segmentFloorPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSegment, pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
    std::vector<pcl::PointIndices::Ptr> segmentWallPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToSegment);
    pcl::PointCloud<pcl::PointXYZ>::Ptr extractPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr inliers);
    double findMinimumDistance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void run();
    void process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    private:
    pcl::visualization::CloudViewer* viewer;
    // The minimum number of inliers for wall segmentation
    static const int INLIER_THRESHOLD = 20000;
    // The maximum distance of a point from a plane model to be considered an inlier
    static const float RANSAC_DISTANCE_THRESHOLD = 0.03;
    // Minimum Pass-Through x-value
    static const float MINIMUM_X = -0.4;
    // Maximum Pass-Through x-value
    static const float MAXIMUM_X = 0.4;
    // Minimum Pass-Through z-value
    static const float MINIMUM_Z = 0.4;
    // Maximum Pass-Through z-value
    static const float MAXIMUM_Z = 4.0;
    // Angle tolerance for a plane to be considered parallel in RANSAC plane segmentation
    static const float ANGLE_THRESHOLD = 15.0;
    // Minimum number of remaining points after floor plane inlier extraction to be considered an obstacle
    static const float OBSTACLE_SIZE_THRESHOLD = 1000;
    // Mean K for Statistical Outlier filter
    static const int MEAN_K = 50;
    // Standard deviation for Statistical Outlier filter
    static const float STANDARD_DEVIATION = 1.0;
    // Voxel cube size for Voxel filter
    static const float LEAF_SIZE = 0.01f;
    // Enable Statistical Outlier filter
    static const bool STATISTICAL_FILTER = false;
    // Enable Voxel filter
    static const bool VOXEL_FILTER = true;
    // Enable Pass-Through filter
    static const bool PASS_THROUGH_FILTER = true;
    // Enable floor segmentation (RANSAC)
    static const bool FLOOR_SEGMENTATION = true;
};

#endif