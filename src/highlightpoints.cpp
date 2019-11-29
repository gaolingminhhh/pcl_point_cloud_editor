#include <pcl/apps/point_cloud_editor/highlightpoints.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <vector>
#include <algorithm>

HightLightPoints::HightLightPoints(CloudPtr cloudptr,SelectionPtr selection_ptr):cloud_ptr(std::move(cloudptr)),selection_ptr_(std::move(selection_ptr))
{
    randomvertex();
    qDebug("初始化计时器!\n");
}

void
HightLightPoints::randomvertex()
{
    int size=cloud_ptr->size();
    qDebug("点云个数%d",size);
    for(int i=0;i<size-70000000;i+=70000000)
    {
        indicies.push_back(i);
    }
}

void
HightLightPoints::getIndicies(IndexVector &index)
{
    index=indicies;
}

void
HightLightPoints::hightlight()
{
    selection_ptr_->addIndex(indicies);
    cloud_ptr->setSelection(selection_ptr_);

}

void
HightLightPoints::dishighlight(int i)
{
        selection_ptr_->removeIndex(i);
}

//void
//HightLightPoints::dishighlight(std::vector<int> removeIndicies)
//{
//    foreach (var i, removeIndicies) {
//        dishighlight(i);
//    }
//}

void
HightLightPoints::highlightsinglepoint(int index)
{
    selection_ptr_->addIndex(index);
}

bool
HightLightPoints::isInIndecies(int num)
{
    if(std::find(selection_ptr_->begin(),selection_ptr_->end(),num)==selection_ptr_->end());
        return false;
    return true;
}
