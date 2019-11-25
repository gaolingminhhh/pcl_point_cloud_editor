#include <pcl/apps/point_cloud_editor/displayDepthValue.h>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/screenpointconverter.h>
#include <QApplication>
#include <QToolTip>
DisplayDepthValue::DisplayDepthValue()
{
}

DisplayDepthValue::~DisplayDepthValue()
{}


//首先应该判断鼠标事件是不是停止
void
DisplayDepthValue::getDepthValue(int x, int y,const QPointF screen_pos,boost::shared_ptr<Converter> convert)
{
    Point3D point;
    if(convert->getDepthValue(x,y,point)){
        qDebug("深度值:%f",point.z);
        QString str("深度值:");
        str.append(QString::number(point.z));
        QToolTip::showText(screen_pos.toPoint(),str);
        // convert->Test(point);
    }
    else
    {
        qDebug("没选中任何点");
    }
}
