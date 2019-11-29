#pragma once

#include <qgl.h>
#include <pcl/apps/point_cloud_editor/toolInterface.h>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <vector>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <pcl/apps/point_cloud_editor/highlightpoints.h>
#include <QPoint>
class Converter{
public:
    Converter(CloudPtr cloud_ptr_,boost::shared_ptr<HightLightPoints> highlight);
    ~Converter();
    //判断点是否在鼠标位置上
    bool isRightPoint(const Point3D& pt,const GLfloat* project,const GLint *viewport) const;
    //得到深度值
    bool getDepthValue(int x, int y,Point3D& point);
    bool getDepthValue(int x, int y,Point3D& point,int& index);

    QPoint getScreenPosValue(Point3D pt);

    void Test(Point3D &point);

    bool isRenderer(Point3D &pt);

    Point3D getObjectSpacePoint (float x,float y,float z) const;

    Point3D getDisplaySpacePoint (float x,float y,float z) const;
private:
    CloudPtr cloud_ptr_;
    //将空间坐标点转换成屏幕坐标点
    //鼠标的坐标位置 xy
    float screen_pos_x;
    float screen_pos_y;
    float mistake_dis=0.01;
    float depth_value;
    boost::shared_ptr<HightLightPoints> highlight;
};
