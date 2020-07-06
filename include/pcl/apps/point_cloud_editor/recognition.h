#ifndef RECOGNITION_H
#define RECOGNITION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/apps/point_cloud_editor/cloud.h>


typedef pcl::PointXYZRGBA PointT;
class Recognition
{
public:
    Recognition(CloudPtr cloud_ptr);
    ~Recognition();
    bool getIndicies(int model);
    bool recognizePlane(float param);
private:
    CloudPtr cloud_ptr;
    void swapRBValues(PclCloudPtr ptr);

};
#endif // RECOGNITION_H
