#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <vector>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/ranging.h>
#include <set>
#include <thread>
class KdtreeSearch
{
public:
    std::vector<pcl::PointXYZRGBA> points;
    std::set<unsigned int> pointindicies;
    KdtreeSearch(CloudPtr cloud_ptr_);
    ~KdtreeSearch();
private:
    CloudPtr cloud_ptr_;
    std::vector<pcl::PointXYZRGBA> similarPoints;
    const unsigned int count=2;
    float length_x=0;
    float length_y=0;
    float length_z=0;
    Point min,max;
    void searchPoint();
};
