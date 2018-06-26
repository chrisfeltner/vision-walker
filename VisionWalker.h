#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>

namespace VisionWalker
{

class VisionWalker
{
    public:
        VisionWalker(){}

        pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter);
        pcl::PCLPointCloud2::Ptr runPassThroughFilter(pcl::PCLPointCloud2::Ptr cloudToFilter, char* field, double min, double max);

    private:
};

}