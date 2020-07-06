#ifndef BOUNDARYESTIMATION_H
#define BOUNDARYESTIMATION_H


#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>

class BoundaryEstimation
{
public:
   static std::vector<unsigned int> getBoundary(PclCloudPtr cloud);
   static std::vector<unsigned int> getBoundary(Cloud3D cloud);
};

#endif // BOUNDARYESTIMATION_H
