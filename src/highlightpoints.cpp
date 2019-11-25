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
HightLightPoints::dishighlight()
{
    if(!isInIndecies(dislightpointindex1)){
        selection_ptr_->removeIndex(dislightpointindex1);
    }
    if(!isInIndecies(dislightpointindex2)){
        selection_ptr_->removeIndex(dislightpointindex2);
    }
}

void
HightLightPoints::highlightsinglepoint(int index)
{
    if(count==2)
    {
        dishighlight();
        count=0;
    }
    if(count==0)
    {
        dislightpointindex1=index;
    }
    if(count==1)
    {
        dislightpointindex2=index;
        //画直线
    }
    selection_ptr_->addIndex(index);
    count++;
}

bool
HightLightPoints::isInIndecies(int num)
{
    if(std::find(indicies.begin(),indicies.end(),num)==indicies.end());
        return false;
    return true;
}
