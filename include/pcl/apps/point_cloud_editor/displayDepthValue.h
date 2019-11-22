#pragma once

#include <qgl.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/screenpointconverter.h>
#include <QPointF>
class DisplayDepthValue
{
public:
    DisplayDepthValue();
    ~DisplayDepthValue();
    //得到深度值
    void getDepthValue(int x, int y,const QPointF pos,boost::shared_ptr<Converter> convert);
private:
    CloudPtr cloud_ptr_;
    //将空间坐标点转换成屏幕坐标点
    //鼠标的坐标位置 xy
    float screen_pos_x;
    float screen_pos_y;
    float mistake_dis=0.01;
};
