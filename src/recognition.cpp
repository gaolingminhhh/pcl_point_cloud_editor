#include <pcl/apps/point_cloud_editor/recognition.h>
#include <QDebug>
#include <pcl/filters/voxel_grid.h>


Recognition::Recognition(CloudPtr cloud_ptr)
{
    this->cloud_ptr=cloud_ptr;
}

Recognition::~Recognition(){}

bool Recognition::getIndicies(int model)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    pcl::PCDWriter writer;
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud_ptr->getCloud3DPtr());
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    //Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (model);
    qDebug()<<"model "<<model;
    seg.setNormalDistanceWeight (0.1f);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.03f);
    seg.setRadiusLimits (0, 0.1f);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    //    writer.write("passthrough.pcd",*cloud_filtered,false);
    //     for(int i=0;i<inliers_plane->indices.size();i++)
    //     {
    //         qDebug()<<i<<":extract point:"<<cloud_plane->points[i].x<<" "<<cloud_plane->points[i].y<<" "<<cloud_plane->points[i].z<<" 源点云"
    //                <<cloud_filtered->points[inliers_plane->indices[i]].x<<" "<<cloud_filtered->points[inliers_plane->indices[i]].y<<" "<<cloud_filtered->points[inliers_plane->indices[i]].z;
    //     }
    if(inliers_plane->indices.size()!=0){
        swapRBValues(cloud_plane);
        writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);//显示出来没有问题 但是索引有问题
        return true;
    }
        qDebug()<<"没有找到";
        return false;

}

void
Recognition::swapRBValues(PclCloudPtr cloud_ptr_)
{
    if (!cloud_ptr_)
        return;
    for (unsigned int i = 0; i < cloud_ptr_ -> size(); i++)
    {
        std::uint8_t cc = (*cloud_ptr_)[i].r;
        (*cloud_ptr_)[i].r = (*cloud_ptr_)[i].b;
        (*cloud_ptr_)[i].b = cc;
    }
}

bool
Recognition::recognizePlane(float param)
{

    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
    pcl::ExtractIndices<PointT> extract;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());

    pcl::PCDWriter writer;
    // Datasets
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices);
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloud_ptr->getCloud3DPtr());
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_filtered);

    //Estimate point normals
    ne.setSearchMethod (tree);
    ne.setInputCloud (cloud_filtered);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setNormalDistanceWeight (0.1f);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (param);
    seg.setRadiusLimits (0, 0.1f);
    seg.setInputCloud (cloud_filtered);
    seg.setInputNormals (cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment (*inliers_plane, *coefficients_plane);

    // Extract the planar inliers from the input cloud
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers_plane);
    extract.setNegative (false);

    // Write the planar inliers to disk
    pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_plane);
    //    writer.write("passthrough.pcd",*cloud_filtered,false);
    //     for(int i=0;i<inliers_plane->indices.size();i++)
    //     {
    //         qDebug()<<i<<":extract point:"<<cloud_plane->points[i].x<<" "<<cloud_plane->points[i].y<<" "<<cloud_plane->points[i].z<<" 源点云"
    //                <<cloud_filtered->points[inliers_plane->indices[i]].x<<" "<<cloud_filtered->points[inliers_plane->indices[i]].y<<" "<<cloud_filtered->points[inliers_plane->indices[i]].z;
    //     }
    if(inliers_plane->indices.size()!=0){
        swapRBValues(cloud_plane);
        writer.write ("table_scene_mug_stereo_textured_plane.pcd", *cloud_plane, false);//显示出来没有问题 但是索引有问题
        return true;
    }
        qDebug()<<"没有找到";
        return false;

}
