#define PCL_NO_PRECOMPILE
# include <pcl/point_types.h>
# include <pcl/point_cloud.h>

using namespace pcl;

namespace extra_point
{
    struct PointNormalContactLabel{
        PCL_ADD_POINT4D;
        PCL_ADD_NORMAL4D;
        float curvature;
        int contact;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;
    POINT_CLOUD_REGISTER_POINT_STRUCT (
        PointNormalContactLabel,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, normal_x, normal_x)
        (float, normal_y, normal_y)
        (float, normal_z, normal_z)
        (float, curvature, curvature)
        (int, contact, contact)
    )
}