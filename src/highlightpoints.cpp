#include <pcl/apps/point_cloud_editor/highlightpoints.h>
#include <pcl/apps/point_cloud_editor/selection.h>
#include <vector>
#include <algorithm>

HightLightPoints::HightLightPoints(CloudPtr cloudptr,SelectionPtr selection_ptr):cloud_ptr(std::move(cloudptr)),selection_ptr_(std::move(selection_ptr))
{
    qDebug("初始化计时器!\n");
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
    cloud_ptr -> drawWithHighlightColor();
}

void
HightLightPoints::dishighlight(int i)
{
        selection_ptr_->removeIndex(i);
        cloud_ptr->setSelection(selection_ptr_);
        cloud_ptr -> draw();
}

void
HightLightPoints::dishighlight(std::vector<unsigned int> removeIndicies)
{
    selection_ptr_->removeIndex(removeIndicies);
    cloud_ptr->setSelection(selection_ptr_);
    cloud_ptr ->draw();
}

void
HightLightPoints::highlightsinglepoint(int index)
{
    selection_ptr_->addIndex(index);
    cloud_ptr->setSelection(selection_ptr_);
    cloud_ptr -> drawWithHighlightColor();
}



bool
HightLightPoints::isInIndecies(int num)
{
    if(std::find(selection_ptr_->begin(),selection_ptr_->end(),num)==selection_ptr_->end());
        return false;
    return true;
}

void
HightLightPoints::highlightpoints(IndexVector indicies)
{
    selection_ptr_->addIndex(indicies);
    cloud_ptr->setSelection(selection_ptr_);
    cloud_ptr -> drawWithHighlightColor();
}

