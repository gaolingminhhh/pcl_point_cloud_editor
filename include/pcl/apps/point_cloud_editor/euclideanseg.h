#ifndef EUCILIDEANSEG_H
#define EUCILIDEANSEG_H
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <vector>
#include <set>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
class EuclideanSeg
{
  public:
    static std::vector<Cloud3D> segmentation(const Cloud3D &cloud);
};

#endif // EUCILIDEANSEG_H
