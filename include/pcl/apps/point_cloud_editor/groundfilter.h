#ifndef GROUNDFILTER_H
#define GROUNDFILTER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <numeric>
#include <iostream>
#include <pcl/io/pcd_io.h>

#include "pcl/octree/octree.h"
#include "pcl/filters/voxel_grid.h"
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/passthrough.h>

#include <iostream>
#include <fstream>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <pcl/common/common.h>
#include <pcl/io/boost.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>



#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <vector>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <pcl/common/transforms.h>

#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <cmath>
#include <algorithm>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>

#define BIG_DATA_POINT_LIMIT 35000
#define DOWNSAMPLE_FACTOR 45
#define CELL_FACTOR 3
#define PI 3.14159265
//#define round(x) ((x < 0) ? (ceil((x)-0.5)) : (floor((x) + 0.5)))

float calc_avg_point_spacing(pcl::PointCloud<pcl::PointXY>& inputCloud);

//下采样
void downsample(const Cloud3D::ConstPtr& input_cloud,
                Cloud3D& downsampled_cloud,
                float leaf_size);

float max_elevation_diff(const Cloud3D& inputCloud);

//坡度
float slope(Cloud3D& cloud, float cell_size) ;

//上采样
void upsample(const Cloud3D::ConstPtr& input_cloud,
              const Cloud3D::ConstPtr& downsampled_cloud,
              Cloud3D& upsampled_cloud,
              float resolution);


void groundFilter(Cloud3D::Ptr& inputCloud,bool isForest,bool isBuilding,Cloud3D::Ptr& groundCloud);


void ground_filter(
        const Cloud3D::ConstPtr& inputCloud,
        Cloud3D& groundCloud,
        float max_distance,
        float cell_size,
        float slope);

#endif // GROUNDFILTER_H
