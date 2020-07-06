#include <pcl/apps/point_cloud_editor/euclideanseg.h>
#include <QDebug>

std::vector<Cloud3D>
EuclideanSeg::segmentation(const Cloud3D &cloudptr)
{
    pcl::PointCloud<Point3D>::Ptr cloud (new pcl::PointCloud<Point3D>), cloud_f (new pcl::PointCloud<Point3D>);
    std::vector<Cloud3D> clouds;
//   // qDebug() << "PointCloud before filtering has: " << cloud->points.size () << " data points." ; //*
    *cloud=cloudptr;

//    // Create the segmentation object for the planar model and set all the parameters
//    pcl::SACSegmentation<Point3D> seg;
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//    pcl::PointCloud<Point3D>::Ptr cloud_plane (new pcl::PointCloud<Point3D> ());
      pcl::PCDWriter writer;
//    seg.setOptimizeCoefficients (true);
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (100);
//    seg.setDistanceThreshold (0.02);

//    int i=0, nr_points = (int) cloud->points.size ();
//    while (cloud->points.size () > 0.3 * nr_points)
//    {
//        // Segment the largest planar component from the remaining cloud
//        seg.setInputCloud (cloud);
//        seg.segment (*inliers, *coefficients);
//        if (inliers->indices.size () == 0)
//        {
//     //       qDebug() << "Could not estimate a planar model for the given dataset." ;
//            break;
//        }

//        // Extract the planar inliers from the input cloud
//        pcl::ExtractIndices<Point3D> extract;
//        extract.setInputCloud (cloud);
//        extract.setIndices (inliers);
//        extract.setNegative (false);

//        // Get the points associated with the planar surface
//        extract.filter (*cloud_plane);
//   //     qDebug() << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." ;
//        clouds.push_back(*cloud_plane);
//        // Remove the planar inliers, extract the rest
//        extract.setNegative (true);
//        extract.filter (*cloud_f);
//        *cloud = *cloud_f;
//    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point3D>::Ptr tree (new pcl::search::KdTree<Point3D>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point3D> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    std::vector<std::vector<int>> pointclouds;
    int j=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        std::vector<int> cloud_cluster ;
        pcl::PointCloud<Point3D>::Ptr cloud_cluster_1 (new pcl::PointCloud<Point3D>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
            cloud_cluster.push_back(*pit); //*            
            cloud_cluster_1->points.push_back (cloud->points[*pit]); //*
        }
        cloud_cluster_1->width = cloud_cluster_1->points.size ();
        cloud_cluster_1->height = 1;
        cloud_cluster_1->is_dense = true;
        clouds.push_back(*cloud_cluster_1);
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<Point3D> (ss.str (), *cloud_cluster_1, false); //*
        pointclouds.push_back(cloud_cluster);
        j++;
    }
 //   qDebug()<<"points cloud count "<<pointclouds.size();
    return clouds;
}
