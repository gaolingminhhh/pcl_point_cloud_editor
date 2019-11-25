#ifndef HIGHLIGHTPOINTS_H
#define HIGHLIGHTPOINTS_H

#include <vector>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <qgl.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>

class HightLightPoints
{
private:
    SelectionPtr selection_ptr_;
    CloudPtr cloud_ptr;
    IndexVector indicies;
    int dislightpointindex1;
    int dislightpointindex2;
    int count=0;
    void dishighlight();
    bool isInIndecies(int num);

public:
    HightLightPoints(CloudPtr cloudptr,SelectionPtr selection_ptr);
    void getIndicies(IndexVector &index);
    void randomvertex();//计算index
    void hightlight();
    void highlightsinglepoint(int index);

};

#endif // HIGHLIGHTPOINTS_H
