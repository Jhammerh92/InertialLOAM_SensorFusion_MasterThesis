#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif
// #include <pcl/memory.h>
// #include <pcl/pcl_macros.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/io/pcd_io.h>

#include <iostream>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h> 
#include <pcl/impl/point_types.hpp>
#include <pcl/impl/instantiate.hpp>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/impl/transforms.hpp>
#include <pcl/registration/icp.h>
#include <pcl/registration/impl/icp.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/features/impl/fpfh.hpp>
#include <pcl/features/pfh.h>
#include <pcl/features/impl/pfh.hpp>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/impl/ia_ransac.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/features/impl/normal_3d.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/impl/approximate_voxel_grid.hpp>
#include <Eigen/Core>


/** \brief Members: float x, y, z, (reflectivity)/intensity, normal[3], curvature, tag, line, timestamp
* \ingroup common
*/
struct EIGEN_ALIGN16 PointXYZRTLTNormal{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    PCL_ADD_NORMAL4D;                 // This adds the member normal[3] which can also be accessed using the point (which is float[4])
    union
    {
        struct
        {
            float intensity;
            float curvature;
            int tag;
            int line;
            float timestamp;
        };
        float data_c[4];
    };
  PCL_MAKE_ALIGNED_OPERATOR_NEW     // make sure our new allocators are aligned
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRTLTNormal,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (float, normal_x, normal_x)
                                    (float, normal_y, normal_y)
                                    (float, normal_z, normal_z)
                                    (float, curvature, curvature)
                                    (int, tag, tag)
                                    (int, line, line)
                                    (float, timestamp, timestamp)
)

PCL_INSTANTIATE(VoxelGrid, PointXYZRTLTNormal)
PCL_INSTANTIATE(NormalEstimation, PointXYZRTLTNormal)
PCL_INSTANTIATE(KdTree, PointXYZRTLTNormal)
PCL_INSTANTIATE(FPFHEstimation,PointXYZRTLTNormal)
PCL_INSTANTIATE(SampleConsensusInitialAlignment, PointXYZRTLTNormal)