#include <pcl/point_types.h>

namespace VisionWalker
{

class VisionWalker
{
    public:
        VisionWalker();

        pcl::PCLPointCloud2::Ptr createVoxelGrid(pcl::PCLPointCloud2::Ptr cloudToFilter = NULL);
        pcl::PCLPointCloud2::Ptr passThroughFilter(pcl::PCLPointCloud2::Ptr cloudToFilter = NULL);

    private:
};

}