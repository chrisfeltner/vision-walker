#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>


class VisionWalker
{
    public:
        VisionWalker(){}

        pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter);
        pcl::PointCloud<pcl::PointXYZ>::Ptr runPassThroughFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudToFilter, char *field, double min, double max);

    private:
};