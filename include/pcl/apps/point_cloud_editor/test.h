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
#define BIG_DATA_POINT_LIMIT 35000
#define PI 3.14159265
#define CELL_FACTOR 3
//#define round(x) ((x < 0) ? (ceil((x)-0.5)) : (floor((x) + 0.5)))
#define MINCRACKDEPTH  2//裂缝的最浅深度
#define KNumbersNeighborOfEdge 30//7-120
#define KNumbersNeighborOfColor 80//7-120


pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRGB2GRAY(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRGB2S(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud) ;


//创建旋转矩阵
Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before,Eigen::Vector3f after);

//计算平均深度值
float getZDepth(pcl::PointCloud<pcl::PointXYZRGBA>& inputCloud);
std::vector<int> getIntensityIndicies(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud_orig );


float calc_avg_point_spacing(pcl::PointCloud<pcl::PointXY>& inputCloud) ;



//max distance
float max_elevation_diff(const pcl::PointCloud<pcl::PointXYZRGBA> &inputCloud);

std::string intToString(int v);


//计算颜色
pcl::PointXYZRGBA findNearestPointsZDepth(std::vector<pcl::PointXYZRGBA> &points);

//计算点云的裂缝
void TagCrackPoint(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud,int nearestCount);
std::vector<int> getEdgeIndex(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
std::vector<int> getColorChangedIndex(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);



int
test();
