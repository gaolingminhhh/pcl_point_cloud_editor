#include <pcl/apps/point_cloud_editor/octreesearch.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <thread>

OctreeSearch::OctreeSearch(CloudPtr cloud_ptr_):cloud_ptr_(cloud_ptr_)
{
    qDebug("初始化八叉树");
    //  cloud_ptr=cloud_ptr_;
    float minx,miny,minz,maxx,maxy,maxz;
    cloud_ptr_->getMax(maxx,maxy,maxz);
    cloud_ptr_->getMin(minx,miny,minz);
    qDebug("max x : %f min x: %f",maxx,minx);
    length_x=(maxx-minx)/count;
    length_y=(maxy-miny)/count;
    length_z=(maxz-minz)/count;
    qDebug("length x: %f, length y:%f,length z:%f",length_x,length_y,length_z);
    for(uint i=0;i<=count;i++)
    {
        for(uint j=0;j<=count;j++)
        {
            for(uint k=0;k<=count;k++)
            {
                pcl::PointXYZRGBA point;
                point.x=i*length_x+minx;
                point.y=j*length_y+miny;
                point.z=k*length_z+minz;
                similarPoints.push_back(point);
            }
        }
    }
    qDebug("count");
    searchPoint();

}

OctreeSearch::~OctreeSearch(){}

void
OctreeSearch::searchPoint()
{
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> octree (256.0f);
    octree.setInputCloud(cloud_ptr_->getCloud3DPtr());
    octree.addPointsFromInputCloud ();
    std::vector<int> vertotint;
    std::vector<float> distance;
    pointindicies.clear();

    for(int i=0;i<similarPoints.size();i++)
    {
        if(!octree.nearestKSearch(similarPoints[i],1,vertotint,distance)>0)
        {
            continue;
        }
        pointindicies.insert(vertotint[0]);
    }
}
