#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>


class VisionWalker
{
    public:
        VisionWalker(){}

        pcl::PointCloud<pcl::PointXYZ>::Ptr createVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFilter);
        pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFilter, char *field, double min, double max);

    private:
};