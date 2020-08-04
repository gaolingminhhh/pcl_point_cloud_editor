#ifndef CONVERTTOIMAGE_H
#define CONVERTTOIMAGE_H


#include "pcl/octree/octree.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/passthrough.h>

cv::Mat makeImageFromPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string dimensionToRemove, float stepSize1, float stepSize2);
pcl::PointCloud<pcl::PointXYZ>::Ptr passThroughFilter1D(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const std::string field, const double low, const double high) ;
#endif
