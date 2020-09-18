#ifndef CRACKIDENTITY_H
#define CRACKIDENTITY_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/io/boost.h>
#include <time.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Eigenvalues>
#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <cmath>
#include <pcl/filters/passthrough.h>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/kdtree/kdtree_flann.h>

//#define round(x) ((x < 0) ? (ceil((x)-0.5)) : (floor((x) + 0.5)))
#define MINCRACKDEPTH  2//裂缝的最浅深度
//#define KNumbersNeighborOfEdge 30//7-120
//#define KNumbersNeighborOfColor 80//7-120

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRGB2GRAY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud) ;
pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRGB2S(Cloud3D::Ptr cloud);
std::vector<unsigned int> getIntensityIndicies(const Cloud3D::Ptr& cloud_orig );
Point3D findNearestPointsZDepth(std::vector<Point3D> &points);
std::vector<unsigned int> getEdgeIndex(const Cloud3D::Ptr& cloud,int knumbersneighborofedge);
std::vector<unsigned int> getColorChangedIndex(const Cloud3D::Ptr& cloud,int knumbersneighborofcolor);
std::vector<unsigned int> crackdetect(const Cloud3D::Ptr& cloud,int knumbersneighborofcolor,int knumbersneighborofedge );
//void crackdetect(const Cloud3D::Ptr& cloud,int k,float thresh,Cloud3D::Ptr& cloud_filtered);
void removeOutlier(Cloud3D::Ptr& cloud,pcl::PointIndices& pointindices);
void transformCloudPoint(const Cloud3D::Ptr& inputCloud,Cloud3D::Ptr& rotatedGround);
float cal_avg_depth(Cloud3D::Ptr& inputCloud);
Eigen::Vector3f getPlane(Cloud3D::Ptr& cloud);
Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before,Eigen::Vector3f after);

bool isNeighbor(Point3D point,Cloud3D::Ptr cloud);

#endif // CRACKIDENTITY_H
