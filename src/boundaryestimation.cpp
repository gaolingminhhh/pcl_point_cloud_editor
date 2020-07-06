#include <pcl/apps/point_cloud_editor/boundaryestimation.h>
#include <QDebug>

std::vector<unsigned int> BoundaryEstimation::getBoundary(PclCloudPtr cloud)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<Point3D,pcl::Normal,pcl::Boundary> est;
    pcl::search::KdTree<Point3D>::Ptr tree(new pcl::search::KdTree<Point3D>());

    pcl::NormalEstimation<Point3D,pcl::Normal> normEst;  //其中Point3D表示输入类型数据，pcl::Normal表示输出类型,且pcl::Normal前三项是法向，最后一项是曲率
    normEst.setInputCloud(cloud);
    normEst.setSearchMethod(tree);
    // normEst.setRadiusSearch(2);  //法向估计的半径
    normEst.setKSearch(20);  //法向估计的点数
    normEst.compute(*normals);

    //normal_est.setViewPoint(0,0,0); //这个应该会使法向一致
    est.setInputCloud(cloud);
    est.setInputNormals(normals);
    //  est.setAngleThreshold(90);
    //   est.setSearchMethod (pcl::search::KdTree<Point3D>::Ptr (new pcl::search::KdTree<Point3D>));
    est.setSearchMethod (tree);
    est.setKSearch(100);  //一般这里的数值越高，最终边界识别的精度越好
    //  est.setRadiusSearch(everagedistance);  //搜索半径
    est.compute (boundaries);
    std::vector<unsigned int> boundariesIndex;
    for (unsigned int i=0; i<cloud->size(); i++){
        uint8_t x = (boundaries.points[i].boundary_point);
        int a = static_cast<int>(x); //该函数的功能是强制类型转换
        if ( a == 1)
        {
            boundariesIndex.push_back(i);
        }
    }
    return boundariesIndex;
}
std::vector<unsigned int>
BoundaryEstimation::getBoundary(Cloud3D cloud)
{
    PclCloudPtr cloud_ptr(new Cloud3D(cloud));
    return getBoundary(cloud_ptr);
}
