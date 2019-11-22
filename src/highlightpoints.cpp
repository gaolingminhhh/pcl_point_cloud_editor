#include <pcl/apps/point_cloud_editor/highlightpoints.h>
#include <pcl/apps/point_cloud_editor/selection.h>

HightLightPoints::HightLightPoints(CloudPtr cloudptr,SelectionPtr selection_ptr):cloud_ptr(std::move(cloudptr)),selection_ptr_(std::move(selection_ptr))
{
    randomvertex();
    qDebug("初始化计时器!\n");
}

void
HightLightPoints::randomvertex()
{
    int size=cloud_ptr->size();
    for(int i=0;i<size-100000;i+=100000)
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
    selection_ptr_->removeIndex(dislightpointindex);
}

void
HightLightPoints::highlightsinglepoint(int index)
{
    dishighlight();
    selection_ptr_->addIndex(index);
    dislightpointindex=index;
}


