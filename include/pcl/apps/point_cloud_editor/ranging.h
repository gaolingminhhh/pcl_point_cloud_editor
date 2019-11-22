#pragma once

#include <qgl.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/screenpointconverter.h>
#include <pcl/apps/point_cloud_editor/rangingdialog.h>
#include <QDialog>
#include <QPushButton>
#include <QLayout>
#include <QLabel>
#include <QGLWidget>

class Ranging
{
public:
    Ranging();
    ~Ranging();
    //TODO2: 根据空间坐标点计算两个点的距离
    float getDistanceOfPoints();
    void getPoint3D(int x,int y,const QPointF screen_pos,boost::shared_ptr<Converter> converter,QWidget *widget);
    void reset();
    void onMouseReleased(int x,int y,const QPointF screen_pos,boost::shared_ptr<Converter> converter,QWidget *widget);
    void onMousePressed(int x,int y);
private:
    //单击鼠标的次数
    int times=0;
    Point3D point1;
    Point3D point2;
    float distance;
    QString point1str;
    QString point2str;
    QString resultstr;
    int final_x;
    int final_y;
    HightLightPoints *highlight;


    void setHighlightColor(Point3D &p);
};
