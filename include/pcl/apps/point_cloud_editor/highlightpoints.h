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
    bool isInIndecies(int num);

public:
    HightLightPoints(CloudPtr cloudptr,SelectionPtr selection_ptr);
    void getIndicies(IndexVector &index);
    void hightlight();
    void dishighlight(int i);
    void dishighlight(IndexVector removeIndicies);
    void highlightsinglepoint(int index);
    void highlightpoints(IndexVector indicies);

};

#endif // HIGHLIGHTPOINTS_H
