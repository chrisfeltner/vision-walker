#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>

#ifndef VISION_WALKER
#define VISION_WALKER

class VisionWalker
{
    public:
        VisionWalker(){}

        pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter);
        pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudToFilter, char *field, double min, double max);
        void run();
        void process(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud);

    private:
};

#endif